#include <hill.h>

namespace Project{
    Hill::Hill(){
        this -> Hill({0.0, 0.0}, 1.0, 1.0)
    }

    Hill::Hill(Eigen::Vector2d pos, double d, double height){
        this -> pos = pos;
        this -> d = d;
        this -> height = height;
    }

    Eigen::Vector2d Hill::gradient(Eigen::Vector2d pos) {
        if ((pos - this->pos).isMuchSmallerThan(0.001)) {
            return {0.0, 0.0};
        }

        double r = (pos - this->pos).norm();
        double module = 0.0;
        Eigen::Vector2d dir = (pos - this->pos) / r;
        if (r <= d / 2) {
            module = -4 * height / (d * d) * r;
        } else if (r <= d) {
            module = 4 * height / (d * d) * r - 4 * height / d;
        } else {
            module = 0.0;
        }

        return dir * module;
    }

    Eigen::Vector2d Hill::get_pos() {
        return this->pos;
    }
}
