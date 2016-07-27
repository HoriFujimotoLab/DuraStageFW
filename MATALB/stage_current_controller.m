clear all;
close all;

%make stage current controller
R=3;
Lq=3.1*10^-3;
ts=125*10^-6; %sampling freq

taue=Lq/R;

%PZ cancel PI
omega0=2*pi*200; %bandwithd[rad/s]
tau=taue;

KP=omega0*Lq;
KI=KP/tau/1.25;

%ss
Gc=KP+tf(1,[1 0])*KI;
ss_ctrl_d=c2d(Gc,ts);

[Afbi, Bfbi, Cfbi, Dfbi] = ssdata(ss_ctrl_d);
%%1
FILENAME = 'current_controler.txt';
FID = fopen(FILENAME,'w');

fprintf(FID,'float\tAfbi\t=\t%.20e;\n',Afbi);
fprintf(FID,'float\tBfbi\t=\t%.20e;\n',Bfbi);
fprintf(FID,'float\tCfbi\t=\t%.20e;\n',Cfbi);
fprintf(FID,'float\tDfbi\t=\t%.20e;\n\n',Dfbi);
fclose(FID);


R=3; %true
Lq=3*10^-3; %true

Gp=tf(1,[Lq R],'InputDelay',10^(-6)*100);
ss_p_d=c2d(Gp,ts);

G=ss_p_d*ss_ctrl_d/(1+ss_p_d*ss_ctrl_d);
bode(G)
figure
step(G)
