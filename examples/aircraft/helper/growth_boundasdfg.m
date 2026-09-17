%% Aircraft DC9-30: Automatic landing maneuver
%% Computation of bounds on the Jacobian

syms g m u1 u2 clear;
syms x1 x2 x3 clear;
f1 = 1/m * (u1 * cos(u2) - (vpa('2.7') + vpa('3.08') * (vpa('1.25') + vpa('4.2') * u2)^2) * x1^2 - m * g * sin(x2));
f2 = 1/(m * x1) * (u1 * sin(u2) + vpa('68.6') * (vpa('1.25') + vpa('4.2') * u2) * x1^2 - m * g * cos(x2));
f3 = x1 * sin(x2);
J1 = hessian(f1, [x1, x2, x3])
J2 = hessian(f2, [x1, x2, x3])
J3 = hessian(f3, [x1, x2, x3])
%% A priori enclosure computed via vnodelp validated ode solution package 
%% (check the ./a_priori_enclosure directory)
%%
X1 = [57.55 83.23];
X2 = [-0.0749 0.0213];
X3 = [-1.37 56.274];

U1 = [0 36000];
U2 = [0 pi*8/180];

l11 = subs(J(1,1),[m,x1],[60000,X1(1)]);
l12 = 9.81;
l21 = subs(J(2,1),[m,g,cos(x2),sin(u2),x1,u1],[60000,9.81,1,-1,83.2,U1(2)]);
l22 = eval(subs(J(2,2),[g, x2,x1],[9.81,X2(2),X1(1)]));
l31 = eval(abs(subs(J(3,1),x2,X2(1))));


L = vpa( [l11 l12 0; l21 l22 0; l31 X1(2) 0] )