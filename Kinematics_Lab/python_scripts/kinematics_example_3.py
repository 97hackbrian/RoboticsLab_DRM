import sympy

#variables simbolicas
x = sympy.symbols('x')
y = sympy.symbols('y')
L = sympy.symbols('L')
#definir angulos de los eslabones
t1=sympy.symbols('t1')
t2=sympy.symbols('t2')
#longitud del eslabon
#L=5

#definicion eslabon1
x=L*sympy.cos(t1)
print(x)
y=L*sympy.sin(t1)
print(y)


#definit valor a L
x_position=x.subs(L,5)
y_position=y.subs(L,5)
print(x_position)
print(y_position)

print(x_position.subs(t1,sympy.pi/8))
print(y_position.subs(t1,sympy.pi/8))

t=sympy.symbols('theta')
sympy.init_printing(use_unicode=True)
print(t)

#variables simbolicas
x = sympy.symbols('x')
y = sympy.symbols('y')
L = sympy.symbols('L')

t=sympy.acos(x/L)
print(t)
angle=t.subs(L,5)
print(angle)
angle=angle.subs(x,4.61939766255643)
print(angle)
angle2=angle.subs(L,5)
print(angle2)
print((angle2*180/sympy.pi).evalf())