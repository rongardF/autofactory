---
name: fix-collada-mesh
description: "Fix the Gazebo gz-common COLLADA segfault caused by VERTEX and NORMAL inputs sharing offset=0 in a <polylist>. Scans one or more .dae files, applies the offset+p-array fix, updates install/ copies, and verifies with a headless gz sim run."
argument-hint: "[path/to/file.dae ...] — omit to scan entire workspace"
allowed-tools: Read, Write, Edit, Bash, Glob, Grep
---

<objective>
Fix the gz-common `ColladaLoader::LoadPolylist` null-deref that causes
`[gazebo-N] Segmentation fault` when loading COLLADA meshes exported by
tools that emit VERTEX and NORMAL inputs both at `offset="0"`.

Root cause (from simulation.md troubleshooting):
- `gz-common` ColladaLoader null-derefs when a `<polylist>` has VERTEX and
  NORMAL inputs sharing `offset="0"`.
- Meshes exported by browser/JS tools (THREE.js GLTFExporter → Assimp) emit
  this shared-offset form.
- Only the render path is affected; physics server + STL collision loads fine.

Fix: give NORMAL its own `offset="1"` and double every index in `<p>` so that
each vertex index `i` becomes the pair `i i` (normals align 1:1 with positions
for these exports).
</objective>

<diagnosis>
Before fixing, check whether a file actually has the bug:

```bash
grep -E '<input .*semantic="(VERTEX|NORMAL)"' path/to/file.dae
# BAD  → both show offset="0"
# GOOD → NORMAL shows offset="1"  (already fixed, skip)
```

To scan the whole workspace for affected files:
```bash
grep -rl 'semantic="NORMAL"' /workspaces/autofactory/src --include="*.dae" \
  | xargs grep -l 'offset="0" semantic="NORMAL"'
```
</diagnosis>

<fix>
Run this Python snippet for each affected file (handles files where the `<p>`
array is a single giant line, which is typical for Assimp exports):

```python
import re, shutil

def fix_dae(path):
    shutil.copy(path, path + '.bak')          # always keep a backup
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()

    # 1. Find the bad NORMAL input
    bad = re.search(r'<input\s+offset="0"\s+semantic="NORMAL"[^>]*/>', text)
    if not bad:
        print(f'SKIP (already good or no NORMAL): {path}')
        return

    # 2. Fix offset: 0 → 1
    text = text.replace(
        bad.group(),
        bad.group().replace('offset="0" semantic="NORMAL"',
                            'offset="1" semantic="NORMAL"'),
        1)

    # 3. Double every index in <p>: "a b c …" → "a a b b c c …"
    p_match = re.search(r'<p>([\d\s]+)</p>', text)
    if not p_match:
        raise RuntimeError(f'No <p> array found in {path}')

    p_ints    = list(map(int, p_match.group(1).strip().split()))
    p_doubled = [x for i in p_ints for x in (i, i)]

    # 4. Validate against <vcount> (stride is now 2)
    vc = re.search(r'<vcount>([\d\s]+)</vcount>', text)
    if vc:
        expected = sum(map(int, vc.group(1).strip().split())) * 2
        assert len(p_doubled) == expected, \
            f'Mismatch: sum(vcount)*2={expected}, len(p)={len(p_doubled)}'

    new_p = f'<p>{" ".join(map(str, p_doubled))}</p>'
    text  = text[:p_match.start()] + new_p + text[p_match.end():]

    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    print(f'Fixed: {path}  ({len(p_ints)} → {len(p_doubled)} indices)')

fix_dae('/path/to/your.dae')
```

After fixing the `src/` copy, update the `install/` copy too — it is NOT a
symlink when the package was built without `--symlink-install` for this asset:

```bash
# Find the matching install copy and overwrite it
SRC=/workspaces/autofactory/src/ur_gazebo_simulation/model/<model>/meshes/<file>.dae
DST=/workspaces/autofactory/install/ur_gazebo_simulation/share/ur_gazebo_simulation/model/<model>/meshes/<file>.dae
cp "$SRC" "$DST"
```
</fix>

<verification>
Run the isolation repro from simulation.md — if the mesh is clean, gz sim
survives for 15 s and exits with SIGTERM (code 143), never SIGSEGV (code 139):

```bash
export DISPLAY=:1   # or your X display; omit for server-only test
cat > /tmp/t.sdf <<'EOF'
<?xml version="1.0"?>
<sdf version="1.9"><world name="t"><model name="m"><static>true</static>
<link name="l"><visual name="v"><geometry><mesh>
<uri>file:///ABS/PATH/to/your_fixed.dae</uri>
</mesh></geometry></visual></link></model></world></sdf>
EOF
source /opt/ros/jazzy/setup.bash
timeout 15 gz sim -s -r -v 2 /tmp/t.sdf 2>&1 | grep -iE 'segfault|segmentation|fault|crash|abort|error'
echo "Exit: $?"   # 143 = clean timeout ✓  |  139 = segfault ✗
```

Also confirm the fix in the file:
```bash
grep -n -E '<input .*semantic="(VERTEX|NORMAL)"' your_fixed.dae
# Expected:
#   offset="0" semantic="VERTEX"  ← unchanged
#   offset="1" semantic="NORMAL"  ← fixed
```
</verification>

<process>
1. Identify which .dae files need fixing (use diagnosis commands above).
2. For each affected file: run the Python fix snippet, then copy to install/.
3. Confirm with the grep check.
4. Run the gz sim isolation test for each fixed mesh.
5. Report: file path, original p-array length → fixed length, vcount validation result.
</process>
