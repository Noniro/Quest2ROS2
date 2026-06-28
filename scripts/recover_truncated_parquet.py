#!/usr/bin/env python3
"""
recover_truncated_parquet.py — salvage a LeRobot dataset after a mid-recording
CRASH/power-loss.

WHY THIS EXISTS
---------------
LeRobot 0.4.x keeps ONE pyarrow `ParquetWriter` open across an entire recording
session and writes one row group per episode (`writer.write_table`). The parquet
FOOTER (schema + row-group offsets) is only written when the writer is CLOSED
(file rollover or clean dataset shutdown). So if the machine dies mid-session,
the current `data/chunk-*/file-NNN.parquet` AND `meta/episodes/chunk-*/file-NNN.parquet`
contain every fully-flushed row group on disk but are unreadable: pyarrow reports
  "Parquet magic bytes not found in footer".
The episodes themselves are NOT lost — only the footer is missing.

WHAT THIS DOES
--------------
Walks the raw page stream of a footer-less parquet, recovers every complete row
group, and rebuilds a valid footer by cloning the schema + per-column templates
from a sibling file in the same dir that DOES have an intact footer (e.g.
file-000). It relies on pyarrow's own decoder for the actual values, so a bad
offset fails LOUD (read error) instead of silently corrupting numbers. It also
self-validates on the intact template (strip → rebuild → byte-identical read)
before touching anything.

USAGE
-----
  # dry-run scan: report which parquet files are broken and recoverable
  python recover_truncated_parquet.py ~/lerobot_datasets/fuel_door

  # actually repair (writes .bak next to each repaired file first)
  python recover_truncated_parquet.py ~/lerobot_datasets/fuel_door --apply

After repair, re-check info.json totals vs the recovered rows (this script
prints them) and load the dataset to confirm. New episodes resume into the NEXT
file index, so repaired files are never rewritten.

Run with the lerobot venv:  ~/lerobot_venv/bin/python recover_truncated_parquet.py ...
"""
import struct, sys, copy, glob, os, argparse

# ---------------- Thrift Compact Protocol: generic schema-agnostic read/write ----------------
CT_STOP=0; CT_TRUE=1; CT_FALSE=2; CT_BYTE=3; CT_I16=4; CT_I32=5; CT_I64=6
CT_DOUBLE=7; CT_BINARY=8; CT_LIST=9; CT_SET=10; CT_MAP=11; CT_STRUCT=12

class Reader:
    def __init__(self, buf, pos=0): self.b=buf; self.p=pos
    def u8(self): v=self.b[self.p]; self.p+=1; return v
    def varint(self):
        shift=0; result=0
        while True:
            byte=self.b[self.p]; self.p+=1
            result |= (byte & 0x7f) << shift
            if not (byte & 0x80): break
            shift+=7
        return result
    def zigzag(self): n=self.varint(); return (n>>1) ^ -(n&1)
    def double(self): v=struct.unpack_from('<d', self.b, self.p)[0]; self.p+=8; return v
    def binary(self): n=self.varint(); v=bytes(self.b[self.p:self.p+n]); self.p+=n; return v
    def read_struct(self):
        fields=[]; last_fid=0
        while True:
            hdr=self.u8()
            if hdr==CT_STOP: break
            ctype=hdr & 0x0f; delta=(hdr & 0xf0)>>4
            fid=self.zigzag() if delta==0 else last_fid+delta
            last_fid=fid
            fields.append([fid,ctype,self.read_value(ctype)])
        return fields
    def read_value(self,ctype):
        if ctype==CT_TRUE:  return True
        if ctype==CT_FALSE: return False
        if ctype==CT_BYTE:  v=self.b[self.p]; self.p+=1; return v if v<128 else v-256
        if ctype in (CT_I16,CT_I32,CT_I64): return self.zigzag()
        if ctype==CT_DOUBLE: return self.double()
        if ctype==CT_BINARY: return self.binary()
        if ctype in (CT_LIST,CT_SET):
            sb=self.u8(); size=(sb>>4)&0x0f; etype=sb&0x0f
            if size==15: size=self.varint()
            return (etype,[self.read_value(etype) for _ in range(size)])
        if ctype==CT_MAP:
            size=self.varint()
            if size==0: return (0,0,[])
            kv=self.u8(); ktype=(kv>>4)&0x0f; vtype=kv&0x0f
            return (ktype,vtype,[(self.read_value(ktype),self.read_value(vtype)) for _ in range(size)])
        if ctype==CT_STRUCT: return self.read_struct()
        raise ValueError(f"bad ctype {ctype} at {self.p}")

class Writer:
    def __init__(self): self.o=bytearray()
    def u8(self,v): self.o.append(v&0xff)
    def varint(self,v):
        v&=(1<<64)-1
        while True:
            b=v&0x7f; v>>=7
            if v: self.o.append(b|0x80)
            else: self.o.append(b); break
    def zigzag(self,n): self.varint((n<<1) ^ (n>>63) if n<0 else (n<<1))
    def double(self,v): self.o += struct.pack('<d',v)
    def binary(self,v):
        if isinstance(v,str): v=v.encode()
        self.varint(len(v)); self.o += v
    def write_struct(self,fields):
        last_fid=0
        for fid,ctype,val in sorted(fields,key=lambda x:x[0]):
            if ctype in (CT_TRUE,CT_FALSE):
                ct = CT_TRUE if val else CT_FALSE
                delta=fid-last_fid
                self.u8((delta<<4)|ct) if 1<=delta<=15 else (self.u8(ct), self.zigzag(fid))
                last_fid=fid; continue
            delta=fid-last_fid
            if 1<=delta<=15: self.u8((delta<<4)|ctype)
            else: self.u8(ctype); self.zigzag(fid)
            last_fid=fid
            self.write_value(ctype,val)
        self.u8(CT_STOP)
    def write_value(self,ctype,val):
        if ctype==CT_BYTE: self.o.append(val&0xff)
        elif ctype in (CT_I16,CT_I32,CT_I64): self.zigzag(val)
        elif ctype==CT_DOUBLE: self.double(val)
        elif ctype==CT_BINARY: self.binary(val)
        elif ctype in (CT_LIST,CT_SET):
            etype,items=val
            if len(items)<15: self.u8((len(items)<<4)|etype)
            else: self.u8((15<<4)|etype); self.varint(len(items))
            for it in items: self.write_value(etype,it)
        elif ctype==CT_MAP:
            ktype,vtype,items=val
            if len(items)==0: self.u8(0)
            else:
                self.varint(len(items)); self.u8((ktype<<4)|vtype)
                for k,v in items: self.write_value(ktype,k); self.write_value(vtype,v)
        elif ctype==CT_STRUCT: self.write_struct(val)
        elif ctype in (CT_TRUE,CT_FALSE): pass
        else: raise ValueError(f"write bad ctype {ctype}")

def get(fields,fid):
    for f in fields:
        if f[0]==fid: return f[2]
    return None
def setf(fields,fid,ctype,val):
    for f in fields:
        if f[0]==fid: f[1]=ctype; f[2]=val; return
    fields.append([fid,ctype,val])
def delf(fields,fid): fields[:] = [f for f in fields if f[0]!=fid]

# ---------------- parquet footer + page walking ----------------
def read_footer(path):
    data=open(path,'rb').read()
    if not (data[:4]==b'PAR1' and data[-4:]==b'PAR1'):
        raise ValueError("not a complete parquet (no PAR1 magic at both ends)")
    flen=struct.unpack_from('<I',data,len(data)-8)[0]
    return Reader(data[len(data)-8-flen:len(data)-8]).read_struct()

def walk_pages(data, start, end):
    p=start
    while p < end:
        r=Reader(data,p)
        try: fields=r.read_struct()
        except Exception: return
        hdr_size=r.p-p
        ptype=get(fields,1); comp=get(fields,3); uncomp=get(fields,2)
        if ptype is None or comp is None: return
        nv=None
        for fh in (5,7,8):
            sub=get(fields,fh)
            if sub is not None: nv=get(sub,1); break
        if p+hdr_size+comp>end: return  # torn body
        yield {'offset':p,'hdr_size':hdr_size,'ptype':ptype,'comp':comp,
               'uncomp':uncomp,'nv':nv,'total':hdr_size+comp}
        p=p+hdr_size+comp

def group_pages(pages, num_cols):
    chunks=[]; cur=[]
    for pg in pages:
        cur.append(pg)
        if pg['ptype'] in (0,3): chunks.append(cur); cur=[]   # data page closes a chunk
    n_rg=len(chunks)//num_cols
    return [chunks[i*num_cols:(i+1)*num_cols] for i in range(n_rg)], len(chunks)-n_rg*num_cols

def build_colmeta(template_cc, chunk_pages):
    cc=copy.deepcopy(template_cc); meta=get(cc,3)
    data_pages=[pg for pg in chunk_pages if pg['ptype'] in (0,3)]
    dict_pages=[pg for pg in chunk_pages if pg['ptype']==2]
    num_values=sum(pg['nv'] for pg in data_pages)
    tot_comp=sum(pg['total'] for pg in chunk_pages)
    tot_uncomp=sum(pg['hdr_size']+(pg['uncomp'] if pg['uncomp'] is not None else pg['comp']) for pg in chunk_pages)
    setf(meta,5,CT_I64,num_values)
    setf(meta,6,CT_I64,tot_uncomp)
    setf(meta,7,CT_I64,tot_comp)
    setf(meta,9,CT_I64,data_pages[0]['offset'])
    if dict_pages: setf(meta,11,CT_I64,dict_pages[0]['offset'])
    else: delf(meta,11)
    for fid in (10,12,13): delf(meta,fid)      # index_page_offset, statistics, encoding_stats
    setf(cc,2,CT_I64,chunk_pages[0]['offset'])  # ColumnChunk.file_offset
    for fid in (4,5,6,7,8): delf(cc,fid)
    return cc, num_values, tot_comp, tot_uncomp

def rebuild_footer(ref_path, broken_data):
    import pyarrow.parquet as pq
    ref_foot = read_footer(ref_path)
    num_cols = pq.ParquetFile(ref_path).metadata.num_columns
    template_rg = get(ref_foot,4)[1][0]
    template_cols = get(template_rg,1)[1]
    assert len(template_cols)==num_cols
    pages=list(walk_pages(broken_data,4,len(broken_data)))
    rowgroups, dropped = group_pages(pages, num_cols)
    if not rowgroups: raise RuntimeError("no complete row groups recovered")
    new_rgs=[]; total_rows=0
    for rg_i,rg_chunks in enumerate(rowgroups):
        cols=[]; nvs=[]; rg_uncomp=0
        for c in range(num_cols):
            cc,nv,tc,tu=build_colmeta(template_cols[c], rg_chunks[c])
            cols.append(cc); nvs.append(nv); rg_uncomp+=tu
        num_rows=min(nvs)
        for c,nv in enumerate(nvs):
            if num_rows==0 or nv % num_rows != 0:
                raise RuntimeError(f"rg{rg_i} col{c} nv={nv} not a multiple of num_rows={num_rows} -> structure mismatch, STOP")
        new_rg=copy.deepcopy(template_rg)
        setf(new_rg,1,CT_LIST,(CT_STRUCT,cols))
        setf(new_rg,2,CT_I64,rg_uncomp)
        setf(new_rg,3,CT_I64,num_rows)
        for fid in (4,5): delf(new_rg,fid)
        new_rgs.append(new_rg); total_rows+=num_rows
    new_foot=copy.deepcopy(ref_foot)
    setf(new_foot,3,CT_I64,total_rows)
    setf(new_foot,4,CT_LIST,(CT_STRUCT,new_rgs))
    w=Writer(); w.write_struct(new_foot); fb=bytes(w.o)
    last=rowgroups[-1][-1][-1]; cut=last['offset']+last['total']
    out = broken_data[:cut] + fb + struct.pack('<I',len(fb)) + b'PAR1'
    return out, total_rows, len(rowgroups), dropped

def is_broken(path):
    import pyarrow.parquet as pq
    try: pq.ParquetFile(path); return False
    except Exception: return True

def find_template(broken_path):
    """An intact sibling parquet in the same dir (same schema)."""
    d=os.path.dirname(broken_path)
    for cand in sorted(glob.glob(os.path.join(d,"*.parquet"))):
        if cand!=broken_path and not is_broken(cand): return cand
    return None

def selftest(ref_path):
    import pyarrow.parquet as pq
    data=open(ref_path,'rb').read()
    flen=struct.unpack_from('<I',data,len(data)-8)[0]
    broken=data[:len(data)-8-flen]
    out,_,_,_=rebuild_footer(ref_path, broken)
    tmp=ref_path+".selftest"; open(tmp,'wb').write(out)
    try:
        ok = pq.read_table(ref_path).equals(pq.read_table(tmp))
    finally:
        os.remove(tmp)
    return ok

def main():
    ap=argparse.ArgumentParser(description="Recover footer-truncated LeRobot parquet files after a crash.")
    ap.add_argument("dataset_root")
    ap.add_argument("--apply", action="store_true", help="actually repair (otherwise dry-run)")
    args=ap.parse_args()
    pats=[os.path.join(args.dataset_root,"data","**","*.parquet"),
          os.path.join(args.dataset_root,"meta","episodes","**","*.parquet")]
    files=sorted(set(sum((glob.glob(p,recursive=True) for p in pats),[])))
    broken=[f for f in files if is_broken(f)]
    print(f"scanned {len(files)} parquet files; {len(broken)} broken")
    if not broken:
        print("nothing to recover."); return
    for bp in broken:
        print(f"\n--- {bp}")
        tmpl=find_template(bp)
        if not tmpl:
            print("  NO intact sibling template in same dir -> cannot rebuild schema. SKIP"); continue
        print(f"  template: {os.path.basename(tmpl)}")
        if not selftest(tmpl):
            print("  SELF-TEST FAILED on template -> refusing to touch this file"); continue
        print("  self-test OK")
        out,rows,n_rg,dropped=rebuild_footer(tmpl, open(bp,'rb').read())
        print(f"  recovered {n_rg} row groups, {rows} rows (dropped {dropped} partial chunks at tail)")
        if args.apply:
            bak=bp+".corrupt.bak"
            if not os.path.exists(bak): os.rename(bp,bak)
            open(bp,'wb').write(out)
            import pyarrow.parquet as pq
            assert pq.ParquetFile(bp).metadata.num_rows==rows
            print(f"  REPAIRED (original saved as {os.path.basename(bak)})")
        else:
            print("  (dry-run; pass --apply to repair)")

if __name__=="__main__":
    main()
