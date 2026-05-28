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

Este repositorio contiene códigos y librerías para implementar controladores para motor DC con microcontrolador. Los valores utilizados están calibrados segun el enconder y especificaciones técnicas del motor utilizado.

Módulos utilizados:
- Driver L298N.
- Current Sensor (Ejemplo: ACS712 / 5A)
- Hall Sensor (Encoder)

## FuzzyLib
Librería en micropython para implementar una lógica difusa. Las funciones contenidas estan diseñadas para no utlizar linspace debido al tiempo de procesamiento que es mayor a 100 ms en tamaños mayores a 50 aproximadamente, lo cual no es recomendable para control de este sistema. Por ello, se recomienda definir las funciones de membresia de entrada correctamente para que la libreria reconozca los extremos como universos correctamente.
Tomar en cuenta que en la función _Defuzzy(membership_out, universe, n)_ se está usando un linspace de tamaño _n_, por lo que hay que equilibrar entre calidad de precisión y tiempo de procesamiento (n = 50 se llega a 20 ms aprox).

```python
def Defuzzy(membership_out, universe, n):
    ...
    # Se establece la defuzificación por método de centroide
    for i in range(n):
        x = universe[0] + i * delta_x 
        
        max_value = 0.0
        
        for membresia in membership_out.values():
            mf_value = Trapzmf(x, membresia)
            max_value = max(max_value, mf_value)
        
        sumy += max_value
        sumy_x += max_value * x
    
    if sumy == 0:
        return 0.0  
    else:
        return sumy_x / sumy
```

## Próximamente
- Observador de Estados en C++ (Arduino) (en especial para sensar corriente de motor cuando el sensor no es eficiente)
- DQN (Aprendizaje por refuerzo)

