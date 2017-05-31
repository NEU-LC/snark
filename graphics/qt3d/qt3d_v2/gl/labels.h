#pragma once
#include "label_shader.h"
#include "types.h"

namespace snark { namespace graphics { namespace qt3d { namespace gl {

class text_label : public label
{
    Eigen::Vector3d position;
    std::string text;
    color_t color;
    int width;
    int height;
public:
    text_label(Eigen::Vector3d position, std::string text,color_t color);
    virtual void update();
protected:
    virtual void draw(QPainter& painter);
};
    
} } } } // namespace snark { namespace graphics { namespace qt3d { namespace gl {
    
