#ifndef ROSUNIT_SUB_HPP
#define ROSUNIT_SUB_HPP

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/Port.hpp"

using std::placeholders::_1;

namespace HEAR{
class ROSUnit_Sub {
protected:
    int id_; 
    Port* _output_port;
public:
    template <class T> OutputPort<T>* getOutputPort() { return (OutputPort<T>*)_output_port;}

    virtual TYPE getType() = 0;
    virtual int getID() const {return id_;}
};
}

#endif
