#ifndef ROSUNIT_PUB_HPP
#define ROSUNIT_PUB_HPP

#include <rclcpp/rclcpp.hpp>
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/Port.hpp"

namespace HEAR{
template<class M>
class ROSUnit_Pub {
protected:
    int id_; 
    rclcpp::Publisher<M>::SharedPtr pub_;
    Port* _input_port;
public:
    template <class T> InputPort<T>* getInputPort() { return (InputPort<T>*)_input_port;}

    virtual TYPE getType() = 0;
    virtual void process() = 0;
    virtual int getID() const {return id_;}
};
}

#endif