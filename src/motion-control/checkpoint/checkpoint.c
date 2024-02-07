#include "checkpoint.h"

namespace Project{
    
    std::ostream& operator << (std::ostream& os, const Checkpoint& cp){
        Eigen::IOFormaat fmt(3, 0, ", ", " \n ", " ", " ");
        os << "[(" << cp.pos.transpose().format(fmt) << ")" << "(" << cp.vel.transpose().format(fmt) << ")" << "]";
        return os;
    }
}