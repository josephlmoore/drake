options = [];
%options.floating = true;
%options.terrain = RigidBodyFlatTerrain();
r = TimeSteppingRigidBodyManipulator('urdf/MPL.urdf',.001, options);
%q = zeros(3,1);
%qd = zeros(3,1);
%[H,C,B,dH,dC,dB] = manipulatorDynamics(r,q,qd);
v = r.constructVisualizer;
v.axis = [-1.7 1.7 -0.1 1.1];

v.display_dt = .05;

x0 = Point(r.getStateFrame);


x0=zeros(52,1);
x0(1)=.1;%randn(1,1);
x0 = resolveConstraints(r,x0);
% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 5],x0);

N=41;

% prog = prog.addStateConstraint(BoundingBoxConstraint([-4,-1,-pi/2,p.phi_lo_limit,-inf,-inf,-inf]',[1,1,pi/2,p.phi_up_limit,inf,inf,inf]'),1:N);
% prog = prog.addStateConstraint(ConstantConstraint(x0),1);
% prog = prog.addStateConstraint(BoundingBoxConstraint([ 0, 0, pi/6 -inf, -2, -2, -inf]',[ 0, 0, 1, inf, 2, 2, inf]'),N);
% 
% prog = prog.addRunningCost(@cost);
% prog = prog.addFinalCost(@(t,x)finalCost(t,x,xf));
% 
% prog = prog.addTrajectoryDisplayFunction(@plotDircolTraj);
% 
% for i=1:5
%   tic
%   [xtraj,utraj,z,F,info,infeasible_constraint_name] = prog.solveTraj(tf0);
%   toc
%   if info==1, break; end
% end
% if info~=1, error('Failed to find a trajectory'); end
% 
% 
%   figure(1)
%   fnplt(xtraj)
%   
%   figure(2)
%   fnplt(utraj)
%   
%   fnplt(xtraj,4)
%   
%   fnplt(xtraj,3)



v.playback(xtraj);
playback(v,xtraj,struct('slider',true));
keyboard