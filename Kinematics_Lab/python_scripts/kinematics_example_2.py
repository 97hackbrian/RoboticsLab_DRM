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

print(sympy.solve(x,t1))
print(sympy.solve(5*sympy.cos(t1)-3,t1))


#Logintud algebraica
H=sympy.sqrt(x**2+y**2)
print(H)
