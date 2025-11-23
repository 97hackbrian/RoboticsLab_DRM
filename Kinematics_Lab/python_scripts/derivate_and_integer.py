import sympy

L=5
t1=sympy.symbols('t1')
x=L*sympy.cos(t1)
dev=sympy.diff(x)
print(dev)


a=sympy.symbols('a')
fa=a**2+5*a+56
dev_fa=sympy.diff(fa)
print(dev_fa)

x=x*(t1+2)
dev_x=sympy.diff(x)
print(dev_x)

j=2*sympy.exp(2*t1)
dev_j=sympy.diff(j)
print(dev_j)

sympy.init_printing(use_unicode=True)
print(dev_j)


print(sympy.integrate(sympy.sin(t1)))
