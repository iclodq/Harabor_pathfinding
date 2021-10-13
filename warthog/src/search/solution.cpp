#include "solution.h"

std::ostream& operator<<(std::ostream& str, warthog::solution& sol)
{
    sol.print_path(str);
    str << std::endl;
    sol.print_metrics(str);
    str << std::endl;
    return str;
}
