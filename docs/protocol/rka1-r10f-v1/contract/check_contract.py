"""Contract-level fixtures only. No production RTL, driver, or hardware is exercised."""
from pathlib import Path
import argparse, binascii, ctypes, hashlib, json, struct, sys
sys.dont_write_bytecode=True
RUN=Path(__file__).resolve().parents[1]
vectors=json.loads((RUN/'contract/acceptance-vectors.json').read_text(encoding='utf-8'))
results=[]
def check(name,actual,expected):
 ok=actual==expected
 results.append({'id':name,'expected':expected,'actual':actual,'pass':ok})
 if not ok: raise AssertionError(name+': '+repr(actual)+' != '+repr(expected))
def crc(data): return binascii.crc32(data)&0xffffffff
def field(b,off,fmt='<I'): return struct.unpack_from(fmt,b,off)[0]
def refresh(b):
 struct.pack_into('<I',b,0x58,0)
 struct.pack_into('<I',b,0x58,crc(b[:128]))
def strict(b,ds):
 if len(b)<128: return False
 if b[:4]!=b'R10F' or (field(b,4,'<H'),field(b,6,'<H'),field(b,8,'<H'))!=(1,0,128): return False
 if b[0x0B]!=0 or field(b,0x0C)!=0 or b[0x2A]!=10 or b[0x2B]!=0: return False
 width,height=field(b,0x1C,'<H'),field(b,0x1E,'<H')
 if width!=ds['width'] or height!=ds['height'] or not width or not height or width%4: return False
 raw=field(b,0x2C); payload=field(b,0x30); stride=field(b,0x20)
 if stride!=width*5//4 or raw!=stride*height or payload!=raw: return False
 if field(b,0x4C)&~2 or any(b[0x5C:128]): return False
 if field(b,0x10)!=0x20261017 or field(b,0x14)!=1: return False
 content=128+payload; dma=(content+4095)//4096*4096
 if dma!=ds['dma_bytes'] or len(b)!=dma or any(b[content:]): return False
 header=bytearray(b[:128]);stored=field(header,0x58);struct.pack_into('<I',header,0x58,0)
 if crc(header)!=stored: return False
 payload_data=b[128:content]
 return crc(payload_data)==field(b,0x50)==field(b,0x54)

check('CRC32-IEEE-check-123456789',crc(b'123456789'),0xCBF43926)
for ds in vectors['datasets']:
 data=(RUN/ds['path']).read_bytes()
 check(ds['name']+'-sha256',hashlib.sha256(data).hexdigest(),ds['sha256'])
 raw=ds['width']*ds['height']*5//4
 content=128+raw; dma=(content+4095)//4096*4096
 for key,value in {'stride_bytes':ds['width']*5//4,'raw_bytes':raw,'payload_bytes':raw,'content_bytes':content,'dma_bytes':dma,'tail_pad':dma-content,'mwr_packets':dma//128,'ddr_read_bursts':dma//4096,'host_stride':(128+dma+4095)//4096*4096,'host_total_alloc':(128+dma+4095)//4096*4096+4096}.items():
  check(ds['name']+'-'+key,value,ds[key])
 group=data[128:133]
 samples=[(group[k]<<2)|((group[4]>>(2*k))&3) for k in range(4)]
 check(ds['name']+'-sample-group',samples,ds['pixel_group'])
 check(ds['name']+'-raw-crc',crc(data[128:content]),ds['raw_crc32'])
 check(ds['name']+'-header-crc',field(data,0x58),ds['header_crc32'])
 for v in vectors['frame_mutations']:
  b=bytearray(data);op=v['operation']
  if op=='none': pass
  elif op=='drop_last_byte': b=b[:-1]
  elif op=='tail_nonzero': b[-1]=1
  elif op=='payload_flip': b[128]^=1
  elif op=='header_flip': b[0x18]^=1
  elif op=='set_codec_1_recrc': struct.pack_into('<I',b,0xC,1);refresh(b)
  elif op=='set_bad_recrc': struct.pack_into('<I',b,0x4C,1);refresh(b)
  elif op=='set_partial_recrc': struct.pack_into('<I',b,0x4C,4);refresh(b)
  elif op=='set_oversize_recrc': struct.pack_into('<I',b,0x4C,8);refresh(b)
  elif op=='set_length_recrc': struct.pack_into('<I',b,0x30,raw+1);refresh(b)
  else: raise ValueError(op)
  check(ds['name']+'-'+v['id'],strict(b,ds),v['expected'])
for v in vectors['control']:
 check(v['id'],(v['readback']&v['required_mask'])==v['required_mask'],v['expected'])
 # Exact pin emulation is only a bit-mask model: bit4 discarded and trigger cleared.
 check(v['id']+'-reserved-read0',v['written']&~0x12,v['readback'])
for v in vectors['dma_boundaries']:
 crosses=[i for i,addr in enumerate(range(v['base'],v['base']+v['bytes'],v['packet_bytes'])) if (addr&4095)+v['packet_bytes']>4096]
 check(v['id'],v['base']%128==0 and not crosses,v['expected'])
 if 'first_crossing_packet_index' in v:
  check(v['id']+'-first-index',crosses[0],v['first_crossing_packet_index'])
  check(v['id']+'-first-address',v['base']+crosses[0]*128,v['first_crossing_address'])
for v in vectors['authorization_cases']:
 check(v['id'],v['authorized'] and v['capacity']>=2592768,v['expected'])

class Mailbox(ctypes.Structure):
 _fields_=[(n,ctypes.c_uint32) for n in ['magic','seq','count','slot_bytes','slots','stride','width','height','push_dbell']]+[('reserved',ctypes.c_uint32*7)]
class Slot(ctypes.Structure):
 _fields_=[(n,ctypes.c_uint32) for n in ['seq','length','flags','reserved']]
check('native-mailbox-push_dbell-offset',Mailbox.push_dbell.offset,32)
check('native-mailbox-size',ctypes.sizeof(Mailbox),64)
check('native-slot-header-size',ctypes.sizeof(Slot),16)

# Arithmetic and state predicates below are formal contract examples, not a model of existing RTL.
def allowed(dst,cap,db,source=0x100000,source_limit=0x20000000,session=1,authorized=True,busy=False,held=False):
 size=2592768
 return bool(authorized and session and not busy and not held and dst and dst%128==0 and cap>=size and dst+size<=2**64 and db and db%4==0 and db+4<=2**64 and not (db<dst+size and db+4>dst) and source%4096==0 and source+size<=source_limit)
good={'dst':0x100000080,'cap':2592768,'db':0x100000000+2596864+32}
check('dma-positive-auth-domain',allowed(**good),True)
for name,changes in [('zero-dst',{'dst':0}),('unaligned-dst',{'dst':0x100000010}),('u64-overflow',{'dst':2**64-128}),('zero-doorbell',{'db':0}),('unaligned-doorbell',{'db':good['db']+1}),('overlapping-doorbell',{'db':good['dst']+128}),('short-cap',{'cap':2592767}),('ungranted',{'authorized':False}),('reset-session',{'session':0}),('busy',{'busy':True}),('held',{'held':True}),('source-misaligned',{'source':0x100001}),('source-high-truncation',{'source':0x20000000}),('source-range-overflow',{'source':0x1ffff000})]:
 check(name,allowed(**(good|changes)),False)
for name,sess,seq,held,expected in [('matching',7,5,True,True),('old-session',6,5,True,False),('wrong-seq',7,4,True,False),('not-held',7,5,False,False)]:
 check('ack-'+name,held and sess==7 and seq==5,expected)
output={'scope':'T03 contract/fixture consistency only; not production RTL, Linux build, G2, or hardware','checks':len(results),'failures':sum(not x['pass'] for x in results),'results':results,'marker':'CONTRACT_FIXTURES_OK'}
parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path);args=parser.parse_args()
if args.out: args.out.write_text(json.dumps(output,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'checks':output['checks'],'failures':output['failures'],'marker':output['marker']}))
