### Car Following 

#### dynamic model:

$$
x_{k+1}=\begin{bmatrix}
1 &\Delta t &-\tau_h\Delta t-r(2v_f-v_{feman})\Delta t\\
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
*Keeping the distance:*
$$
\Delta d-(r(v_f-v_{fmean})+\tau_h)\Delta v\ge -\tau_hv_p+d_{s0}-d_0
$$

*which can be converted to*:
$$
-\Delta d+(r(v_p-\Delta v-v_{fmean})+\tau_h)\Delta v -\tau_hv_p+d_{s0}-d_0\leq 0
$$


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

#### Example:

$$
NP=2\\
\ \\
x=[*s_0,u_0,*s_1,u_1,*s_2]\\
x_{max}=[*s_0,u_{max},*s_{max},u_{max},*s_{max}]\\
x_{min}=[*s_0,u_{min},*s_{min},u_{min},*s_{min}]\\
\ \\
g=[s_1-f(s_0),h(s_1),s_2-f(s_1),h(s_2)]\\
g_{max}=[*0,0,*0,0]\\
g_{min}=[*0,-\infty,*0,-\infty]\\
\ \\
L=l(u_0,s_1)+l(u_1,s_2)
$$

*if using CBF*:
$$
g=[s_1-f(s_0),h(s_1),s_2-f(s_1),h(s_2)]\\
g_{max}=[0,(1-\lambda)h(s_0),0,(1-\lambda)h(s_1)]\\
g_{min}=[0,-\infty,0,-\infty]
$$
