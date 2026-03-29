# NASA Power of Ten Rules for Safety-Critical Code

These rules were developed by Gerard J. Holzmann of the NASA Jet Propulsion Laboratory (JPL) to ensure the reliability of mission-critical software.

1.  **Simple Control Flow**: Restrict all code to very simple control flow constructs – do not use `goto` statements, `setjmp` or `longjmp` constructs, or direct or indirect recursion.
2.  **Fixed Loop Bounds**: All loops must have a fixed upper-bound. It must be trivially possible for a static tool to prove that a preset bound on the number of iterations of a loop cannot be exceeded.
3.  **No Dynamic Allocation**: Do not use dynamic memory allocation after initialization. This includes `malloc`, `free`, `calloc`, and `realloc`.
4.  **Function Length**: No function should be longer than what can be printed on a single sheet of paper in a standard reference format with one line per statement and one line per declaration (approx. 60 lines).
5.  **Assertion Density**: The assertion density of the code should average to a minimum of two assertions per function. Assertions are used to check for anomalous conditions that should never happen in real-life executions.
6.  **Small Data Scope**: Declare data objects at the smallest possible level of scope.
7.  **Check Return Values**: The return value of non-void functions must be checked by each calling function, or the return value must be explicitly cast to `void` if it is not used.
8.  **Limited Preprocessor**: The use of the preprocessor must be limited to the inclusion of header files and simple macro definitions. Token pasting, variable number of arguments (varargs), and recursive macro calls are not allowed.
9.  **Pointer Limit**: Use of pointers should be restricted. Specifically, no more than one level of dereferencing is allowed. Pointer arithmetic is not allowed.
10. **Zero Warnings**: All code must be compiled, from the first day of development, with all compiler warnings enabled at the compiler’s most pedantic setting. All code must compile with zero warnings.
