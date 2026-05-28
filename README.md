# PIDz_circuitpython
Librería para PID discreto para Circuitpython. Está implementado en un objeto que actualiza los errores pasados y las variables de control dentro de una ecuación de diferencias.

## v1.0

### 
```python
class PID:
    def __init__(self):
        self.reset()
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.Ts = 0
        self.update()
```
Esta biblioteca de Python proporciona una implementación de controlador PID adecuada para sistemas de control discretos en microcontroladores. El controlador permite ajustar dinámicamente las ganancias proporcionales (Kp), integrales (Ki) y derivadas (Kd) sin llamar explícitamente a una función de actualización. La biblioteca utiliza el módulo numpy de la biblioteca ulab de Circuitpython para realizar cálculos numéricos eficientes.

En esta versión el controlador emplea solo el método de integración trapezoidal para el control PID en tiempo discreto.

$$
c[k] = c[k-1] + q_0 \cdot e[k] + q_1 \cdot e[k-1] + q_2 \cdot e[k-2]
$$

Donde: 

$$
\mathbf{q} = \begin{bmatrix}
q_0 \\
q_1 \\
q_2 \\
\end{bmatrix} 
= \begin{bmatrix}
Kp + \frac{Ki \cdot Ts}{2} + \frac{Kd}{Ts} \\
-Kp + \frac{Ki \cdot Ts}{2} - \frac{2 \cdot Kd}{Ts} \\
\frac{Kd}{Ts} \\
\end{bmatrix}
$$
