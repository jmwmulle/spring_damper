"""Lossless compact encoding: unchanged parameters are stored once per family."""
import json,sys,gzip
from pathlib import Path
root=Path(__file__).resolve().parents[2]
scalar=json.loads(Path(sys.argv[1]).read_text())
for name,rows in list(scalar.items()):
 template=rows[0][0];columns=[i for i in range(len(template)) if any(row[0][i]!=template[i] for row in rows)]
 scalar[name]={'template':template,'columns':columns,'rows':[[[r[0][i] for i in columns],r[1]] for r in rows]}
(root/'tests/goldens/scalar.json.gz').write_bytes(gzip.compress((json.dumps(scalar,separators=(',',':'))+'\n').encode(),mtime=0))
quat=json.loads(Path(sys.argv[2]).read_text())
# Output state equals returned output except for update (5); other unchanged states are already in the inputs.
for r in quat:
 if r[0]!=5: del r[7:]
(root/'tests/goldens/quat.json.gz').write_bytes(gzip.compress((json.dumps(quat,separators=(',',':'))+'\n').encode(),mtime=0))
