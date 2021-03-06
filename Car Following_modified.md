### Car Following 

#### dynamic model:

*continuous form*
$$
\dot{x}=\begin{pmatrix}0& 1& -\tau_h-r(2v_f-v_fmean)\\0&0&-1\\0&0&-1/T_G\end{pmatrix}x+\begin{pmatrix}0\\0\\K_G/T_G\end{pmatrix}u+\begin{pmatrix}0\\1\\0\end{pmatrix}v
$$
*discrete form*
$$
x_{k+1}=\begin{pmatrix}
1 &\Delta t &-\tau_h\Delta t-r(2v_f-v_{feman})\Delta t\\
&1 &-\Delta t\\
&&1-\Delta t/T_G
\end{pmatrix}
x_k+\begin{pmatrix}0\\0\\\Delta t\cdot K_G/T_G\end{pmatrix}u
+\begin{pmatrix}0\\\Delta t\\0\end{pmatrix}v
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

​		which is derived from
$$
\Delta d+d_{des} \ge d_{s0} \ or \ TTC * \Delta V
$$
​		desired distance is from 
$$
d_{\text {des}}=r v_{f}\left(v_{f}-v_{\text {fmean}}\right)+\tau_{h} v_{f}+d_{0}
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
K_G=1.05\\ T_G=0.393s\\\tau_h=1.68s\\r=0.054 s^2m^{-1}\\
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

表示：

1. cbf可以和长时间的ptw有同样的安全性能，但短时间的ptw可能会撞。

   三者的预测时域（cost）一样

2. 长时间ptw缺点：前车加速度可观测，那么长时间ptw有缺点

   三条线：长ptw,短ptw,短cbf。想说明跟踪效果好

3. 优化包：ipopt blocksqp 别的包。计算时间序列，每一步。不同的作比较。箱型图。