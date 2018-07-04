function cmd_pos=V_o(pos_err,k,eps,r_M,r_a,r_s,r_o)
    t=normal(pos_err);
    r_Mo = r_M+r_o;
    r_ao = r_a+r_o;
    f = k*sig(t,r_Mo,r_ao);
    f_grad = k*grad_sig(t,r_Mo,r_ao);
    g = (1 +eps) * t - r_Mo * s_m(t/r_Mo, r_s);
    g_grad = (1 + eps) - grad_s_m(t/r_Mo, r_s);
    V = (f_grad*g-f*g_grad)/(g*g);
    cmd_pos = V*(pos_err)/t;
end

function sig_out=sig(x,  d1, d2)
     A = -2/((d1-d2)*(d1-d2)*(d1-d2));
     B =  3*(d1+d2)/((d1-d2)*(d1-d2)*(d1-d2));
     C = -6*d1*d2/((d1-d2)*(d1-d2)*(d1-d2));
     D = d2*d2*(3*d1-d2)/((d1-d2)*(d1-d2)*(d1-d2));

    if (x<d1)
        sig_out = 1.0;
    elseif (x>d2)
        sig_out= 0.0;
    else
        sig_out= A*x*x*x+B*x*x+C*x+D;
    end
end

function grad_sig_out=grad_sig( x,  d1, d2)

    A = -2/((d1-d2)*(d1-d2)*(d1-d2));
    B =  3*(d1+d2)/((d1-d2)*(d1-d2)*(d1-d2));
    C = -6*d1*d2/((d1-d2)*(d1-d2)*(d1-d2));
    D = d2*d2*(3*d1-d2)/((d1-d2)*(d1-d2)*(d1-d2));

    if (x<d1)
        grad_sig_out = 1.0;
    elseif (x>d2)
        grad_sig_out= 0.0;
    else
        grad_sig_out=  3*A*x*x+2*B*x+C;
    end
end

function s_m_out=s_m(x, rs)

    x2 = 1+rs*(1/tan(67.5/180*pi));
    x1 = x2 - rs*sin(45/180*pi);

    if (x>=0 && x<x1)
        s_m_out = x;
    elseif (x >x2)
        s_m_out= 1.0;
    else
        s_m_out = (1-rs)+sqrt(rs*rs-(x-x2)*(x-x2));
    end
end

function grad_s_m_out=grad_s_m(x, rs)
    x2 = 1+rs*(1/tan(67.5/180*pi));
    x1 = x2 - rs*sin(45/180*pi);

    if (x>=0 && x<x1)
        grad_s_m_out = 1.0;
    elseif (x >x2)
        grad_s_m_out = 0.0;
    else
        grad_s_m_out= (x2-x)/sqrt(rs*rs-(x-x2)*(x-x2));
    end
end