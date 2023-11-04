## GAMES101 Assignment Solutions

### Assignment 1

The assignment is all about formula implementation. It includes two parts,

- Construct the rotation matrix w.r.t. z axis for model space-to-view space transformation
- Construct the perspective projection matrix

According to what taught in GAMES 101 lectures, the formula for the first part is,
```math
R_z (\alpha) = 
\begin{pmatrix}
\cos \alpha & - \sin \alpha & 0 & 0 \\
\sin \alpha & \cos \alpha & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{pmatrix}
```
Therefore, the code should be (remember to transform the input angle from degree to radius),

```c++
Eigen::Matrix4f get_model_matrix(float rotation_angle) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    float rad_angle = DEG2RAD (rotation_angle);

    Eigen::Matrix4f rot_mat;
    rot_mat << cos (rad_angle), - sin (rad_angle), 0, 0,
               sin (rad_angle), cos (rad_angle), 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;

    model = rot_mat * model;
    
    return model;
}
```



The second part is about projection, which is divided into two distinct components,

- Transforming perspective projection to orthographic projection

$$
M_{persp \rightarrow ortho} =
\begin{pmatrix}
n & 0 & 0 & 0 \\
0 & n & 0 & 0 \\
0 & 0 & n + f & - nf \\
0 & 0 & 1 & 0
\end{pmatrix}
$$

- Do orthographic projection transformation (translate & scale)
  ```math
  M_{ortho} =
  \begin{bmatrix}
  \frac2{r - l} & 0 & 0 & 0 \\
  0 & \frac2{t - b} & 0 & 0 \\
  0 & 0 & \frac2{n - f} & 0 \\
  0 & 0 & 0 & 1
  \end{bmatrix}
  \begin{bmatrix}
  1 & 0 & 0 & - \frac{r + l}2 \\
  0 & 1 & 0 & - \frac{t + b}2 \\
  0 & 0 & 1 & - \frac{n + f}2 \\
  0 & 0 & 0 & 1
  \end{bmatrix}
  ```

It is easy to do the first part, but finds $l, r, b, t$ unknown to do the second one

<img src="https://ooo.0x0.ooo/2023/11/03/ONeWht.png" alt="ONeWht.png" style="zoom:80%;" />

We can, however, figure out value of those parameters according to the above formulas

```c++
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    Eigen::Matrix4f persp2orth_mat;
    persp2orth_mat << zNear, 0, 0, 0,
                      0, zNear, 0, 0,
                      0, 0, zNear + zFar, - zNear * zFar,
                      0, 0, 1, 0;

    float t = - zNear * tan (DEG2RAD (eye_fov) / 2.0f);
    float b = - t;
    float r = t * aspect_ratio;
    float l = - r;

    Eigen::Matrix4f orth_translate_mat, orth_scale_mat;
    orth_translate_mat << 1, 0, 0, - (r + l) / 2,
                      0, 1, 0, - (t + b) / 2,
                      0, 0, 1, - (zNear + zFar) / 2,
                      0, 0, 0, 1;
    orth_scale_mat << 2 / (r - l), 0, 0, 0,
                      0, 2 / (t - b), 0, 0,
                      0, 0, 2 / (zNear - zFar), 0,
                      0, 0, 0, 1;
    Eigen::Matrix4f orth_mat = orth_scale_mat * orth_translate_mat;

    projection = orth_mat * persp2orth_mat * projection;

    return projection;
}
```



---

### Assignment 2

#### 1 - Create Bounding Box

It is easy to implement, just find $(x_{min}, y_{min})$ and $(x_{max}, y_{max})$

```C++
int x_min = INT_MAX, y_min = INT_MAX, x_max = INT_MIN, y_max = INT_MIN;
for (int i = 0; i < 3; i ++) {
    x_min = std::min (x_min, (int) t.v[i].x ());
    x_max = std::max (x_max, (int) t.v[i].x ());
    y_min = std::min (y_min, (int) t.v[i].y ());
    y_max = std::max (y_max, (int) t.v[i].y ());
}
```

#### 2 - Check If A Certain Pixel is Inside the Triangle

Assume a triangle with vertices $A, B, C$ and a point $P$. In order to determine if $P$ is inside triangle $ABC$, what we need to check is whether $P$ is on the same side of vectors $\overrightarrow{AB}, \overrightarrow{BC}, \overrightarrow{CA}$. Here we take advantage of the property of cross product. We can guarantee that $P$ is on the same side as long as those cross products have the same sign

The implementation below make use of $z$ values in results of `Vector3` cross products, for $z' = x_1y_2 + x_2y_1$, i.e., the cross product of two `Vector2`

```C++
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    Vector3f dot = Vector3f (x, y, 1.0f);

    Vector3f tri0_1 = _v[1] - _v[0], tri1_2 = _v[2] - _v[1], tri2_0 = _v[0] - _v[2];
    Vector3f tri0_dot = dot - _v[0], tri1_dot = dot - _v[1], tri2_dot = dot -_v[2];

    Vector3f cross0 = tri0_1.cross (tri0_dot), cross1 = tri1_2.cross (tri1_dot), cross2 = tri2_0.cross (tri2_dot);

    return cross0.z () * cross1.z () >= 0.0f && cross0.z () * cross2.z () >= 0.0f;
}
```

#### 3 - Maintain Z-Buffering

The same as the logic taught in lectures

```c++
for (int x = x_min; x <= x_max; x ++)
    for (int y = y_min; y <= y_max; y ++)
        if (insideTriangle (x + 0.5f, y + 0.5f, t.v)) {
            // Interpolation
            auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;

            // Z-Buffering Maintainance
            if (depth_buf[get_index (x, y)] > z_interpolated) {
                depth_buf[get_index (x, y)] = z_interpolated;
                set_pixel (Vector3f (x, y, z_interpolated), t.getColor ());
            }
        }
```

#### 4 - MSAA

As we can see from the result of above codes, the triangles are successfully painted, but have alias. It reminds us to do some anti-aliasing. Here we adopt *supersampling*, which divides a pixel into four fewer parts, and samples them respectively. For simplicity, assuming a pixel located in $(x, y)$, we pick $(x + 0.25, y + 0.25), (x + 0.25, y + 0.75), (x + 0.75, y + 0.25), (x + 0.75, y + 75)$ to implement supersampling

```C++
std::vector<Eigen::Vector3f> frame_buf;
std::vector<Eigen::Vector3f> ss_frame_buf; // Supersamepling

std::vector<float> depth_buf;
std::vector<float> ss_depth_buf; // Supersampling
int get_index(int x, int y);
int ss_get_index (int x, int y, int id); // Supersampling

int width, height;
int ss_width, ss_height; // Supersampling
```

Additional frame buffer and depth buffer are required. It is easy to write in code as we can imitate what have been written

```C++
// Inside rasterizer::clear
if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    std::fill (ss_frame_buf.begin (), ss_frame_buf.end (), Eigen::Vector3f {0, 0, 0});
if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    std::fill (ss_depth_buf.begin (), ss_depth_buf.end (), std::numeric_limits<float>::infinity ());
```

```C++
// Inside rasterizer constructor
ss_width = 4 * w, ss_height = h;
ss_frame_buf.resize (4 * w * h);
ss_depth_buf.resize (4 * w * h);
```

```C++
int rst::rasterizer::ss_get_index (int x, int y, int id) {
    int ss_x = x * 4 + id, ss_y = y;
    return (ss_height - 1 - ss_y) * ss_width + ss_x;
}
```

Now the code for pixel color prediction should be changed to

```C++
const float delta[4][2] = { {0.25f, 0.25f}, {0.25f, 0.75f}, {0.75f, 0.25f}, {0.75, 0.75f} };

for (int x = x_min; x <= x_max; x ++)
    for (int y = y_min; y <= y_max; y ++) {
        int insideCounter = 0;

        for (int k = 0; k < 4; k ++) {
            // per sub-pixel
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
            // Adopt a average color of the four sub-pixels
            Vector3f color = (ss_frame_buf[ss_get_index (x, y, 0)] + ss_frame_buf[ss_get_index (x, y, 1)] +
                              ss_frame_buf[ss_get_index (x, y, 2)] + ss_frame_buf[ss_get_index (x, y, 3)]) / 4;
            set_pixel (Vector3f (x, y, 0.0f), color);
        }
    }
```

#### 5 - Result

<img src="https://ooo.0x0.ooo/2023/11/04/ONve9t.jpg" style="zoom: 50%;" />
