import numpy
import sympy
import math as m
#print(m.pi)
#print(sympy.sqrt(5)*sympy.pi)

x=[5,9,8,5]
x = sympy.symbols('x')
y = sympy.symbols('y')

fx=2*x
fxy=(2*x+y)*y
fn=fx+fxy+x+y+sympy.sqrt(-1)*x
print(sympy.expand(fn))

print(sympy.factor(fn))

pol=x**2+x*5+6
print(sympy.factor(pol))

c=sympy.Rational(0.2).limit_denominator(10000)
print(c)

fx=x**2+x*2+2
print(fx)


fx=x**2+3*x+2
fx_x=sympy.solve(fx,x)
print(fx_x)


fx2=x**2+2*x+2
fx_x2=sympy.solve(fx2,x)
print(fx_x2)

fxy=2*x+2*y
fxy_x=sympy.solve(fxy,x)
print(fxy_x)

