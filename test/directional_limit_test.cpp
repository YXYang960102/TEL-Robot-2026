#include <assert.h>

#include "../src/Control/DirectionalLimit.h"

int main() {
    assert(DirectionalLimit::apply(0.75, false, false) == 0.75);
    assert(DirectionalLimit::apply(-0.75, false, false) == -0.75);
    assert(DirectionalLimit::apply(0.0, true, true) == 0.0);

    assert(DirectionalLimit::apply(-0.5, true, false) == 0.0);
    assert(DirectionalLimit::apply(0.5, true, false) == 0.5);

    assert(DirectionalLimit::apply(0.5, false, true) == 0.0);
    assert(DirectionalLimit::apply(-0.5, false, true) == -0.5);

    assert(DirectionalLimit::apply(-1.0, true, true) == 0.0);
    assert(DirectionalLimit::apply(1.0, true, true) == 0.0);
    return 0;
}
