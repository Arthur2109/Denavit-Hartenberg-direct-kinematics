

function T=cambio_ij(MDH,i,j) %cualquier i y j, con i<j.
Tij=eye(4,4);
for k=i+1:j  % ejemplo :i=3, j=6 ---> k=4 hasta 6 ---> T46=T34*T45*T56 --->OK
    Tij=Tij*T_ij_vecinos(k,MDH);

    end
T=Tij;
end

function T=fn_THDH(eje,ang,q)
    switch eje
        case 'x'
            R=[1,       0,        0;
               0,cos(ang),-sin(ang);
               0,sin(ang),cos(ang)];
        case 'y'
            R=[cos(ang), 0,sin(ang);
               0,        1,       0;
               -sin(ang),0,cos(ang)];
        case 'z'
            R=[cos(ang),-sin(ang),0;
               sin(ang), cos(ang),0;
               0,         0      ,1];
        case 't'
            R=[1, 0, 0;
               0, 1, 0;
               0, 0, 1];
    end

    T=sym(eye(4,4));
    T(1:3,4)=q;
    T(1:3,1:3)=R;
end

function T=T_ij_vecinos(j,DH)
t = num2cell(DH(j,:));
[aij alfa sj thj] = deal(t{:});
T=fn_THDH('t',0,[aij,0,0])*fn_THDH('x',alfa,[0 0 0])*fn_THDH('t',0,[0 0 sj])*fn_THDH('z',thj,[0 0 0]);
end
