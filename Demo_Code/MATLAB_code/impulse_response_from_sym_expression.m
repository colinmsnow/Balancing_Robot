function h = impulse_response_from_sym_expression(H)

    TFstr = char(H);

    % Define 's' as transfer function variable
    s = tf('s');
    % Evaluate the expression: "TF = (s+2)/(s^2+5*s+9)"
    eval(['TFH = ',TFstr]);
    impulse(TFH);

end