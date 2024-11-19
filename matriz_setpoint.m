
function SlnSetPoint=matriz_setpoint(DH,q06,s06,a067)


a01=DH(1,1);
a12=DH(2,1);
a23=DH(3,1);
a34=-DH(4,1);
S1=-DH(1,3);
S4=-DH(4,3);
S6=-DH(6,3);

%definición de los parametros del robot.


SlnSetPoint=eye(6,8);


%Definicion del primer punto 
q06=transpose(q06);
s06=transpose(s06); %z
a067=transpose(a067);%x
y06=cross(s06,a067);%y

Rf6=[a067,y06,s06];

Tf6=sym(eye(4,4));
Tf6(1:3,1:3)=Rf6;
Tf6(1:3,4)=q06;
Tf6(4,:)=[0,0,0,1];

P6tool=transpose([0,0,-120]);

Pfc_right=q06-Rf6*P6tool+S6*s06; %posición del punto de interes C en el fijo.

px=Pfc_right(1);
py=Pfc_right(2);
pz=Pfc_right(3);

%% 

%CALCULO DE THETA 1
th1a=atan(-py/px);
th1b=th1a+pi;
th1 = [th1a, th1b];

%th1a=atan(-py/px)
%th1b=th1a+pi
%% 

%CALCULO DE THETA 3
A=-2*a23*a34;
B=-2*S4*a23;
h=(A^2+B^2)^0.5;
alpha=atan2(B/h,A/h);

D=[- S1^2 + 2*S1*pz + S4^2 - DH(2,1)^2 + 2*cos(th1(1))*a12*px - 2*sin(th1(1))*a12*py + a23^2 + a34^2 - px^2 - py^2 - pz^2,- S1^2 + 2*S1*pz + S4^2 - a12^2 + 2*cos(th1(2))*a12*px - 2*sin(th1(2))*a12*py + a23^2 + a34^2 - px^2 - py^2 - pz^2];

th3=[acos(-D(1)/h)+alpha,wrapTo2Pi(2*pi-(acos(-D(1)/h)+alpha)),acos(-D(2)/h)+alpha,wrapTo2Pi(2*pi-(acos(-D(2)/h)+alpha))];
%% 

%CALCULO DE THETA 2
A1=NaN(2,2,4);
B1=NaN(2,1,4);
for i=1:size(th3,2)
    A1(:,:,i)=[a23-a34*cos(th3(i))-S4*sin(th3(i)), a34*sin(th3(i))-S4*cos(th3(i));
               S4*cos(th3(i))-a34*sin(th3(i)),a23-a34*cos(th3(i))-S4*sin(th3(i)) ];
end

B1=[px*cos(th1a)-py*sin(th1a)-a12,-pz+S1;px*cos(th1b)-py*sin(th1b)-a12,-pz+S1];



sol_th2=zeros(2,4);
th2=[0;0;0;0];
for i=1:4
    sol_th2(:,i)=A1(:,:,i)\B1(floor(i/3)+1,:)';
    th2(i)=atan2(sol_th2(2,i),sol_th2(1,i));
end

th2;
%% 
%CALCULO DE THETA 5

% 1) Calculo de R36 Left con la funcion DH de 6 en el 3:
T36l=cambio_ij(DH,3,6);
R36l=T36l(1:3,1:3); % =R36 left
b23l=R36l(2,3) ;% =cos(theta5) % este calculo no sirve porque ya sabemos en valor de b23l. Solo para comprobar.

% 2) Calculo de R36 Right con Tf6 y post-multiplicación :
% Tf6=Tf1*T12*T23*T34*T45*T56
% T32*T21*T1f*Tf6=T23*T34*T45*T56
% T32*T21*T1f*Tf6=T36 Right=T36 left
% Tf6 es conocido. Falta calcular T32*T21*T1f=T3f=inv(cambio_f3)

T3f=inv(cambio_ij(DH,0,3));
T36r=T3f*Tf6;
R36r=T36r(1:3,1:3); % =R36 right;
b23r=R36r(2,3); % =function(th1,th2,th3)=cos(th5)
b23r=expand(simplify(b23r)); % = cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2) = cos(th1)*sin(th2+th3)=cos(theta5)
% th5a=arcos (cos(th1)*sin(th2+th3)) y th5b=2pi-th5a

th5a=acos(cos(th1a)+sin(th2(1)+th3(1)));
th5b=2*pi-th5a;
th5c=acos(cos(th1a)+sin(th2(2)+th3(2)));
th5d=2*pi-th5c;
th5e=acos(cos(th1b)+sin(th2(3)+th3(3)));
th5f=2*pi-th5e;
th5g=acos(cos(th1b)+sin(th2(4)+th3(4)));
th5h=2*pi-th5g;

Th5=[th5a,th5b,th5c,th5d,th5e,th5f,th5g,th5h];
disp(class(Th5));

for i=1:8
    if isreal(Th5(i))==0;
        Th5(i)=NaN;
    end
end


%% CALCULO DE THETA 6
b22r=R36r(2,2);% tipo symbolic (porque función de th1 th2 y th3 que son listas)
b21r=-R36r(2,1) ;

Th6=[0,0,0,0,0,0,0,0];
Th1=th1;
Th2=th2;
Th3=th3;
%Th5=th5; % Th5 ya definido arriba

for i=1:8
    th1=Th1(floor((i-1)/4)+1);
    th2=Th2(floor((i-1)/2)+1);
    th3=Th3(floor((i-1)/2)+1);
    % Utilización de la función subs para afectar a b22r y b21r valores de scalar
%    disp(subs(b21r)/sin(Th5(i)));
    Th6(1,i)=atan2(subs(b22r)/sin(Th5(i)),subs(b21r)/sin(Th5(i)));
    % b22 y b21 todavia son listas en este momento gracias a subs
end

%% CALCULO DE THETA 4
b13r=R36r(1,3); % tipo symbolic (porque función de th1 th2 y th3 que son listas)
b33r=R36r(3,3) ;

SlnSetPoint(1,:)=[Th1(1),Th1(1),Th1(1),Th1(1),Th1(2),Th1(2),Th1(2),Th1(2)];
SlnSetPoint(2,:)=[Th2(1),Th2(1),Th2(2),Th2(2),Th2(3),Th2(3),Th2(4),Th2(4)];
SlnSetPoint(3,:)=[Th3(1),Th3(1),Th3(2),Th3(2),Th3(3),Th3(3),Th3(4),Th3(4)];
SlnSetPoint(5,:)=Th5;
SlnSetPoint(6,:)=Th6;

Th4=[0,0,0,0,0,0,0,0];


for i=1:8
    th1=Th1(floor((i-1)/4)+1);
    th2=Th2(floor((i-1)/2)+1);
    th3=Th3(floor((i-1)/2)+1);
    % Utilización de la función subs para afectar a b22r y b21r valores de scalar
    Th4(1,i)=atan2(b33r/sin(Th5(i)),b13r/sin(Th5(i)));

    % b22 y b21 todavia son listas en este momento gracias a subs
    if isnan(Th4(1,i)) %Si un elemento de una columna es NaN, Cambiamos todos los valores de esta columna por NaN
        SlnSetPoint(:,i)=NaN;
    end
end


SlnSetPoint(4,:)=Th4;

end



%{
Pfc_right=[px,py,pz,1]'; %parte 1 de la ecuación (derecha)
Pfc_right=inv(cambio_ij(DH,0,1))*Pfc_right;
Pc3=[-a34,-S4,0,1]';
Pfc_left=cambio_ij(DH,1,3)*Pc3
Pfc_right=simplify(expand(Pfc_right))
Pfc_left=simplify(expand(Pfc_left));
%}




