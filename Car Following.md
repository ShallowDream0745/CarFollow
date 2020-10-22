### Car Following 

#### dynamic model:

$$
x_{k+1}=\begin{bmatrix}
1 &\Delta t &-\tau_h\Delta t-r(2v_f-v_{feman}\Delta t)\\
&1 &-\Delta t\\
&&1-\Delta t/T_G
\end{bmatrix}
x_k+\begin{bmatrix}0\\0\\\Delta t\cdot K_G/T_G\end{bmatrix}u
$$

where
$$
\begin{aligned}
x=\begin{bmatrix}\Delta d\\\Delta v\\a_f\end{bmatrix}&=\begin{bmatrix}d-d_{safe}\\v_p-v_f\\a_f\end{bmatrix}\\
u&=a_{fdes}
\end{aligned}
$$

#### Loss Function:

$$
L(x,u)=\sum_{i=1}^{N}(w_d\Delta d^2+w_v\Delta v^2+w_aa_{fdes}^2)
$$



#### Subj.to:

*Control Constraint:*
$$
a_{fmin}\leq a_{fdes} \leq a_{fmax}
$$
*Keep the distance:*
$$
\Delta d-(r(v_f-v_{fmean})+\tau_h)\Delta v\ge -\tau_hv_p+d_{s0}-d_0
$$

#### 

#### Expected distance:

$$
d_{des}=rv_f(v_f-v_{fmean})+\tau_hv_f+d_0\\=(r(v_f-v_{fmean}）+\tau_h)v_f+d_0
$$

#### Parameters:

$$
K_G=1.05\\ T_G=0.393s\\\tau_h=1.68s\\
\Delta t=0.1s\\\ \\ w_d=0.8\\ w_v=1 \\ w_a=5\\\ \\
TTC=-2.5s\\ d_{s0}=5m\\ d_0=2.9m
\\a_{fdes}\in[-1.5,0.5] m/s^2
$$

