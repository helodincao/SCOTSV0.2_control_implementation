%
% hvac.m
%
% created: Feb 2017
%  author: rungger
%

function hvac

clear set
close all


%% simulation
% initial state
%x0=[-8 0 8 0]
x0=[3 0 0.1 0];

% sampling time (in sec)
dt=.25;
% simulation horizon (in time steps) simulate for half a minute
N=100;

% load the symbolic set containing the controller
C=StaticController('cartpole_sparse');

%% sample-and-hold loop
y=x0;
v=[];
for t=1:N

  % pick input
  u=C.control(y(end,:));

  if(t==1)
    idx=1;
  else
    %% different strategies to determinize controller
    switch(2)
      case 1, % minimum energy input
        [uu idx]=min(sum(u.^2,2));
      case 2, % last input if possible
        uold=v(end,:);
        [uu idx]=min(sum((u-repmat(uold,size(u,1),1)).^2,2));
      case 3,
        idx=1;
    end
  end

  v=[v; u(idx,:)];
  [tt x]=ode45(@ode,[(t-1)*dt t*dt], y(end,:), odeset('abstol',1e-4,'reltol',1e-4),v(end,:));

  if t<N
    y=[y; x(end,:)];
  end
end
t=(0:1:N-1)*dt;

%% plot the dcdc safe set
% colors
colors=get(groot,'DefaultAxesColorOrder');

%% plot initial state  and trajectory
figure
plot(t,y(:,1),'.-')
figure
plot(t,y(:,2),'.-')
figure
plot(t,y(:,3),'.-')
figure
plot(t,y(:,4),'.-')
%plot(t,y,'.-')

%% plot inputs
figure 
plot(t,v)


end

function dxdt = ode(t,x,u)
  omega=1;
  ga=0.01251;

  dxdt(1) = x(2);
  dxdt(2) = -omega*omega*(sin(x(1))+u*cos(x(1)))-2*ga*x(2);
  dxdt(3) = x(4);
  dxdt(4) = u-x(4);

  dxdt=dxdt(:);
end

