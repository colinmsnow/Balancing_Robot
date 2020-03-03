syms s a b l g Kp Ki Jp Ji Ci  % define symbolic variables

Htot = 0;  % place your transfer function here

%% Substitute parameters and solve

% system parameters
g = 9.85;
l = 22*2.54/100;
l = 0.4217
a = 14;
b = 1/400;

Htot_subbed = subs(Htot); % substitutes parameters defined above into Htot

% define the target poles

p1 = 
p2 = 
p3 = 
p4 = 
p5 = 

% this is the target characteristic polynomial
tgt_char_poly = (s-p1)*(s-p2)*(s-p3)*(s-p4)*(s-p5);

% get the denominator from Htot_subbed
[n d] = numden(Htot_subbed);

% find the coefficients of the denominator polynomial TF
coeffs_denom = coeffs(d, s);

% divide out the coefficient of the highest power term
coeffs_denom = coeffs(d, s)/(coeffs_denom(end));

% find coefficients of the target charecteristic polynomial
coeffs_tgt = coeffs(tgt_char_poly, s);

% solve the system of equations setting the coefficients of the
% polynomial in the target to the actual polynomials
solutions = solve(coeffs_denom == coeffs_tgt, Jp, Ji, Kp, Ki, Ci);

% display the solutions as double precision numbers

Jp = double(solutions.Jp)
Ji = double(solutions.Ji)
Kp = double(solutions.Kp)
Ki = double(solutions.Ki)
Ci = double(solutions.Ci)

impulse_response_from_sym_expression(subs(Htot))