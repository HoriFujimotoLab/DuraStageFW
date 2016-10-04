clear all;
close all;

%make stage velocity controller
ts=1*10^-3; %sampling period

%P
omega0=2*pi*1; %bandwithd[rad/s]

KP=omega0;

%ss
Gc=KP*tf(1,1);
ss_ctrl_d=c2d(Gc,ts);

[Afbi, Bfbi, Cfbi, Dfbi] = ssdata(ss_ctrl_d);
%%1
FILENAME = 'stage_position_controler.txt';
FID = fopen(FILENAME,'w');

fprintf(FID,'float Avpi[2][1]={{%.20e}, {%.20e}};\n',Afbi, Afbi);
fprintf(FID,'float Bvpi[2][1]={{%.20e}, {%.20e}};\n',Bfbi, Bfbi);
fprintf(FID,'float Cvpi[2][1]={{%.20e}, {%.20e}};\n',Cfbi, Cfbi);
fprintf(FID,'float Dvpi[2][1]={{%.20e}, {%.20e}};\n',Dfbi, Dfbi);
fclose(FID);


%{
R=3; %true
Lq=3*10^-3; %true

Gp=tf(1,[Lq R],'InputDelay',10^(-6)*100);
ss_p_d=c2d(Gp,ts);

G=ss_p_d*ss_ctrl_d/(1+ss_p_d*ss_ctrl_d);
bode(G)
figure
step(G)
%}