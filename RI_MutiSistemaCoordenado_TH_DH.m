%% Direct Kinematic Model With DH and Graph
clear all
clc
close all

%% Preliminar

% sistema coordenado fijo
hf=figure(1);
set(hf,'position',[445   275   563   628])
L=70;
h_ejexF=plot3([0 L],[0 0],[0 0],'r','linewidth',2.5);
hold on
h_ejeyF=plot3([0 0],[0 L],[0 0],'g','linewidth',2.5);
hold on
h_ejezF=plot3([0 0],[0 0],[0 L],'b','linewidth',2.5);
hold on

axis equal
axis([-100 1500 -600 600 -100 1500])
grid on
xlabel('X [-]')
ylabel('Y [-]')
zlabel('Z [-]')
view(27,37) % genera una orientacion de la vista.

Mp=[0,0,0,1;  % origen
    L,0,0,1;  % eje x
    0,L,0,1;  % eje y
    0,0,L,1]; % eje z

% Matriz DH definida sin la columna de los th porque son variables.
th1=3;
th2=2;
th3=0;
th4=0;
th5=1;
th6=-1;

DH=[   0    pi  -675 th1;   %F1  
       260  pi/2   0  th2;   %12
     680     0   0  th3;   %23
    -35  pi/2  -670 th4;   %34
       0  pi/2   0  th5;   %45
       0  pi/2  -115 th6];  %56

% Sistemas coordenados moviles n=6
for i=1:6
    c=cambio_ij(DH,0,i);
    o=c*Mp(1,:)';
    x=c*Mp(2,:)';
    y=c*Mp(3,:)';
    z=c*Mp(4,:)';
    h_ejexM(i,1)=plot3([o(1) x(1)],[o(2) x(2)],[o(3) x(3)],'r','linewidth',2);
    hold on
    h_ejeyM(i,1)=plot3([o(1) y(1)],[o(2) y(2)],[o(3) y(3)],'g','linewidth',2);
    hold on
    h_ejezM(i,1)=plot3([o(1) z(1)],[o(2) z(2)],[o(3) z(3)],'b','linewidth',2);
end
 
%% 
% Matriz de P. DH
%DH=[a_ij alpha_ij s_j theta_j]

Q=zeros(25,3);
P=zeros(25,3);
S6 = [];
a6 = [];
P(1,:) = [800, 0, 700];
%size(Q,1)
for i=2:25
    
    if i<11
        P(i,:) = [P(i-1,1)+40, 0, 700];      
        S6(i,:) = [-1 0 0];
        a6(i,:) = [0 0 1];
    elseif 10<i<21
        P(i,:) = [1160, P(i-1,2)+40, 700];
        S6(i,:) = [0 1 0];
        a6(i,:) = [0 0 1];
    else
        P(i,:) = [1160, 400, P(i-1,10)+40];
        S6(i,:) = [0 0 -1];
        a6(i,:) = [0 1 0];
    end
end


%lista de matrices de cada punto de forma P(p)=[[q;s;a],[q;s;a]...]
SlnSet=[];

for p=1:size(P)
    display(p);
    SlnSetPoint=matriz_setpoint(DH,P(p,:),S6(p,:),a6(p,:));
    for k=1:8

        fila=SlnSetPoint(:,k)';
        SlnSet(p,:,k)=fila;
    end
end

for p=1:size(P,3)
    display("test");
    DH(:,4)=SlnSet(p,:,1)';
    DHp=DH;
    T=L_Matriz_06(DHp); %valable para 1 punto

    % %k=1 T_f1; k=2 T_f2; k=3 T_f3; k=4 T_f4
    for k=1:DoF
        for i=1:4
            Mp_T(i,:,k)=(T(:,:,k)*Mp(i,:)')'; 
        end
    end
    % 
    for k=1:DoF 
        set(h_ejexM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(2,1,k)],...
                         'ydata',[Mp_T(1,2,k) Mp_T(2,2,k)],...
                         'zdata',[Mp_T(1,3,k) Mp_T(2,3,k)])
    
        set(h_ejeyM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(3,1,k)],...
                         'ydata',[Mp_T(1,2,k) Mp_T(3,2,k)],...
                         'zdata',[Mp_T(1,3,k) Mp_T(3,3,k)])
    
        set(h_ejezM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(4,1,k)],...
                         'ydata',[Mp_T(1,2,k) Mp_T(4,2,k)],...
                         'zdata',[Mp_T(1,3,k) Mp_T(4,3,k)])
    end
end