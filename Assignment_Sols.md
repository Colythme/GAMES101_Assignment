## GAMES101 Assignment Solutions

### Assignment 1

The assignment is all about formula implementation. It includes two parts,

- Construct the rotation matrix w.r.t. z axis for model space-to-view space transformation
- Construct the perspective projection matrix

According to what taught in GAMES 101 lectures, the formula for the first part is,
$$
R_z (\alpha) = 
\begin{pmatrix}
\cos \alpha & - \sin \alpha & 0 & 0 \\
\sin \alpha & \cos \alpha & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{pmatrix}
$$
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
  $$
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
  $$

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

