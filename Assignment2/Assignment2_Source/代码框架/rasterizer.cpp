// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    Vector3f dot = Vector3f (x, y, 1.0f);

    Vector3f tri0_1 = _v[1] - _v[0], tri1_2 = _v[2] - _v[1], tri2_0 = _v[0] - _v[2];
    Vector3f tri0_dot = dot - _v[0], tri1_dot = dot - _v[1], tri2_dot = dot -_v[2];

    Vector3f cross0 = tri0_1.cross (tri0_dot), cross1 = tri1_2.cross (tri1_dot), cross2 = tri2_0.cross (tri2_dot);

    return cross0.z () * cross1.z () >= 0.0f && cross0.z () * cross2.z () >= 0.0f;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // (x_mim, y_min), (x_max, y_max)
    int x_min = INT_MAX, y_min = INT_MAX, x_max = INT_MIN, y_max = INT_MIN;
    for (int i = 0; i < 3; i ++) {
        x_min = std::min (x_min, (int) t.v[i].x ());
        x_max = std::max (x_max, (int) t.v[i].x ());
        y_min = std::min (y_min, (int) t.v[i].y ());
        y_max = std::max (y_max, (int) t.v[i].y ());
    }

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    const float delta[4][2] = { {0.25f, 0.25f}, {0.25f, 0.75f}, {0.75f, 0.25f}, {0.75, 0.75f} };

    for (int x = x_min; x <= x_max; x ++)
        for (int y = y_min; y <= y_max; y ++) {
            int insideCounter = 0;

            for (int k = 0; k < 4; k ++) {
                if (insideTriangle (x + delta[k][0], y + delta[k][1], t.v)) {
                    auto[alpha, beta, gamma] = computeBarycentric2D(x + delta[k][0], y + delta[k][1], t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    insideCounter ++;
                    if (ss_depth_buf[ss_get_index (x, y, k)] > z_interpolated) {
                        ss_depth_buf[ss_get_index (x, y, k)] = z_interpolated;
                        ss_frame_buf[ss_get_index (x, y, k)] = t.getColor ();
                    }
                }
            }

            if (insideCounter > 0) {
                Vector3f color = (ss_frame_buf[ss_get_index (x, y, 0)] + ss_frame_buf[ss_get_index (x, y, 1)] +
                                  ss_frame_buf[ss_get_index (x, y, 2)] + ss_frame_buf[ss_get_index (x, y, 3)]) / 4;
                set_pixel (Vector3f (x, y, 0.0f), color);
            }
        }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }

    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
        std::fill (ss_frame_buf.begin (), ss_frame_buf.end (), Eigen::Vector3f {0, 0, 0});
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
        std::fill (ss_depth_buf.begin (), ss_depth_buf.end (), std::numeric_limits<float>::infinity ());
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    ss_width = 4 * w, ss_height = h;
    ss_frame_buf.resize (4 * w * h);
    ss_depth_buf.resize (4 * w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::ss_get_index (int x, int y, int id) {
    int ss_x = x * 4 + id, ss_y = y;
    return (ss_height - 1 - ss_y) * ss_width + ss_x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on