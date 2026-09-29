#pragma once

// Blocking bench tests; caller must require testing mode and a waiting round.
// F: drive 500 mm according to pose. O: one full turn in each direction.
// P: motors-off manual push, 30 seconds or X to finish; prints raw counts.
void flow_test_run(char command);
