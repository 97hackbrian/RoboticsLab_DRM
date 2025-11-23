import sympy

#definir angulos de los eslabones
t1=sympy.symbols('t1')
t2=sympy.symbols('t2')
#longitud del eslabon
L=5
#definicion de un eslabon
x=L*sympy.cos(t1)
print(x)
y=L*sympy.sin(t1)
print(y)

#Logintud algebraica
H=sympy.sqrt(x**2+y**2)
print(H)

#sustituir valor de angulo
print(H.subs(t1,0))
print(H.subs(t1,sympy.pi/2))
print(H.subs(t1,666))

#obligar a calcular valor numerico
print(H.subs(t1,666).evalf())



