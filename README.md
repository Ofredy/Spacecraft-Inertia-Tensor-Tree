## Spacecraft-Inertia-Tensor-Tree

Spacecraft Inertia Tensor Tree

Compute the moment of inertia (MOI) for a modular spacecraft assembled as a tree of rigid modules.
Each module has its own mass points; the tree wires modules together with a relative transform (quaternion + offset).
A depth-first (post-order) pass accumulates children first, then parents, up to the root.

Key features

Load a station from JSON (modules, mass points, per-edge transforms).

Per-module:

calculate_module_inertia() → local mass, COM, and MOI about the module’s COM.

calculate_module_inertia_wrt_parent() → rotate to parent frame and parallel-axis shift using subtree mass.

DFS utility to:

compute every child first,

fold children into each parent’s inertia_com_,

yield the root’s total MOI about the root COM.


Station Used in Code:

Total modules: 6 (a root; children b,c,d,e; f is child of d)
Mass per module: 4 points × 500 kg = 2000 kg
Total mass: 12,000 kg

                          ( +y )
                            ↑
                            |
                (-5,+5)     |    (+5,+5)
                   b        |       c
                 roll 60°   |    roll 20°
                      \     |     /
                       \    |    /
                        \   |   /
                         \  |  /
                           module_a (root)
                         /  / \  \
                        /  /   \  \
                       /  /     \  \
            (-5,-5)  d  |        \  e  (+5,-5)
           roll 80°     |         \ roll 70°
                        |
                      (0,-5)
                        f
                      roll 0°
Legend:

Labels on edges are parent_offset [dx,dy] from child COM to parent COM.

“roll θ°” means the quaternion given is a rotation about x by θ (per your data).

All modules have identical internal point layout at (±1, ±1, 0).
