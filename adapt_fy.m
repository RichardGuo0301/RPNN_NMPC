function [sys,x0,str,ts] = spacemodel(t,x,u,flag)
switch flag,
    case 0,
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 1,
        sys=mdlDerivatives(t,x,u);
    case 3,
        sys=mdlOutputs(t,x,u);
    case {2,4,9}
        sys=[];
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end
function [sys,x0,str,ts]=mdlInitializeSizes
global a
global b
a=rand(1,2*24)*5;
b=rand(1,24)*2;
sizes = simsizes;
sizes.NumContStates  = 24*2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 8;%vy,gamma的模型值真实值和前一时刻的模型值真实值
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = [0*ones(24*2,1)];

str = [];
ts  = [];
function sys=mdlDerivatives(t,x,u)
global a
global b

gama=0.5;%更新率里面的权重
ka=0.8;%更新率里面的权重

%% 输入数据
% ek_vx=u(1);
ek_vy=u(1);%当前时刻真实的vy
ek_gamma=u(2);%当前时刻真实的gamma
ek1_vy=u(3);%前一时刻真实的vy
ek1_gamma=u(4);%前一时刻真实的gamma
ek_vy_model = u(5);%当前时刻模型的vy
ek_gamma_model = u(6);%当前时刻模型的gamma
ek1_vy_model = u(7);%前一时刻时刻模型的vy
ek1_gamma_model = u(8);%前一时刻模型的gamma

xi=[ek_vy_model ek_gamma_model]';
input_history=[ek1_vy-ek1_vy_model ek1_gamma-ek1_gamma_model]';
input=[ek_vy-ek_vy_model ek_gamma-ek1_gamma_model]';
alpha=zeros(1,24);
% W=[];

for j=1:1:24
    alpha(1,j)=1/(1+exp(-(a(1,(j-1)*2+1:j*2)*xi+b(1,j))));
end
H=[alpha,zeros(1,24);zeros(1,24),alpha];
% for i=1:24*2
%     W=[W;x(i)];
% end
S=gama*H'*(input-ka*input_history);

for i=1:1:48
    sys(i)=S(i,1);
end

function sys=mdlOutputs(t,x,u)

global a 
global b
% a=rand(1,2*24)*20;
% b=rand(1,24)*0.5;
% ek_vx=u(1);
ek_vy=u(1);
ek_gamma=u(2);

% ek1_vx=u(4);
ek1_vy=u(3);
ek1_gamma=u(4);
input=[ek_vy-ek1_vy ek_gamma-ek1_gamma]';
% a=rand(1,2*24);
% b=rand(1,24);
 W=[x(1) x(2) x(3) x(4) x(5) x(6) x(7) x(8) x(9) x(10) x(11) x(12) x(13) x(14) x(15) x(16) x(17) x(18) x(19) x(20) x(21) x(22) x(23) x(24) x(25) x(26) x(27) x(28) x(29) x(30) x(31) x(32) x(33) x(34) x(35) x(36) x(37) x(38) x(39) x(40) x(41) x(42) x(43) x(44) x(45) x(46) x(47) x(48)]';

alpha=zeros(1,24);
for j=1:1:24
    alpha(1,j)=1/(1+exp(-(a(1,(j-1)*2+1:j*2)*input+b(1,j))));
end
H=[alpha,zeros(1,24);zeros(1,24),alpha];
fxp=H*W;

sys(1)=fxp(1);
sys(2)=fxp(2);

