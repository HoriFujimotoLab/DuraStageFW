clear all;
close all;

%make stage current controller
R=3;
Lq=3.1*10^-3;
ts=200*10^-6; %sampling freq

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

%input shaping
Gc_ff=tf(1,[1/omega0/5, 1]); %4000 [Hz] LPF
ss_ctrl_d_ff=ss(c2d(Gc_ff,ts));
[Affi, Bffi, Cffi, Dffi] = ssdata(ss_ctrl_d_ff);
%%1
fprintf(FID,'float\tAffi\t=\t%.20e;\n',Affi);
fprintf(FID,'float\tBffi\t=\t%.20e;\n',Bffi);
fprintf(FID,'float\tCffi\t=\t%.20e;\n',Cffi);
fprintf(FID,'float\tDffi\t=\t%.20e;\n',Dffi);
fclose(FID);


%R=3; %true
%Lq=3*10^-3; %true

R=3;
Lq=13*10^-3;

Gp=tf(1,[Lq R],'InputDelay',10^(-6)*10);
ss_p_d=c2d(Gp,ts);

G=ss_p_d*ss_ctrl_d/(1+ss_p_d*ss_ctrl_d)*ss_ctrl_d_ff;
bode(G)
figure
step(G)
