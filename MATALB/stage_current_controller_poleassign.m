clear all;
close all;

%stage controller
BANDWIDTH=800;
ts=125*10^-6; %sampling freq

%Plant
R=3;
Lq=3.1*10^-3;
Gp=tf(1,[Lq R]);

sys_p=ss(Gp);
[Ap,Bp,Cp,Dp]=ssdata(sys_p);

ps=[-2*pi*BANDWIDTH];
K=place(Ap,Bp,ps);

%{

taue=Lq/R;

%PZ cancel PI
omega0=2*pi*800; %bandwithd[rad/s]
tau=taue;

KP=omega0*Lq;
KI=KP/tau;

%ss
Gc=KP+tf(1,[1 0])*KI;
ss_ctrl_d=ss(c2d(Gc,ts));

[Afbi, Bfbi, Cfbi, Dfbi] = ssdata(ss_ctrl_d);
%%1
FILENAME = 'current_controler.txt';
FID = fopen(FILENAME,'w');

fprintf(FID,'float\tAfbi\t=\t%.20e;\n',Afbi);
fprintf(FID,'float\tBfbi\t=\t%.20e;\n',Bfbi);
fprintf(FID,'float\tCfbi\t=\t%.20e;\n',Cfbi);
fprintf(FID,'float\tDfbi\t=\t%.20e;\n\n',Dfbi);
fclose(FID);


%R=3; %true
%Lq=3*10^-3; %true

Gp=tf(1,[Lq R],'InputDelay',10^(-6)*100);
ss_p_d=c2d(Gp,ts);

G=ss_p_d*ss_ctrl_d/(1+ss_p_d*ss_ctrl_d);
bode(G)
figure
step(G)
%}