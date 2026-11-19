# C module templates

Copy [module.h.txt](module.h.txt) and [module.c.txt](module.c.txt), then replace
the module name, include guard, purpose and ownership/contracts. The `.txt`
suffixes keep examples outside compiled source. These support BARR-C 4.4.a;
they do not replace a review of an existing module's interface and behavior.

Keep source sections in this order: module comment, includes, types/constants/
macros, static objects, private prototypes, public definitions, private definitions.
Document each private function as well as public interfaces. Preserve license
notices when applying the structure to an existing file.
