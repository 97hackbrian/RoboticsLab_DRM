import sympy
from sympy import sin, symbols, cos, pi

#variables simbolicas
x = sympy.symbols('x')
y = sympy.symbols('y')
L1 = sympy.symbols('L_1')
L2 = sympy.symbols('L_2')
#definir angulos de los eslabones
t1=sympy.symbols('tetha_1')
t2=sympy.symbols('tetha_2')
#longitud del eslabon

x=L1*cos(t1)+L2*cos(t1+t2)
print(x)
y=L1*sin(t1)+L2*sin(t1+t2)
print(y)

x_position=x.subs({L1:5,L2:3})
print(x_position)

y_position=y.subs({L1:5,L2:3})
print(y_position)




#print(x_position.subs({t1:pi/4,t2:pi/8}))
#print(y_position.subs({t1:pi/4,t2:pi/8}))



px=symbols('px')
py=symbols('py')

eq1=x_position-px
print(eq1)

eq2=y_position-py
print(eq2)

solution=sympy.solve([eq1,eq2],(t1,t2))
print(solution)