classdef QPControllerData < ControllerData
  % Class that contains data needed by the QPController and cascaded modules.

  % properties that change infrequently or never
  properties (SetAccess=private,GetAccess=public)
    % linear dynamics (default: ZMP) ---------------------------------------------
    A = [zeros(2),eye(2); zeros(2,4)] % COM state map
    B = [zeros(2); eye(2)] % COM input map
    C = [eye(2),zeros(2)] % ZMP state-output map
    D % ZMP input-output map

    A_is_time_varying=false 
    B_is_time_varying=false 
    C_is_time_varying=false 
    D_is_time_varying=false 
  end
  
  % properties that can be modified 'on the fly'
  properties (SetAccess=public,GetAccess=public)
    % solver related -------------------------------------------------------------
    infocount=0 % number of consecutive iterations with solver info < 0
    qp_active_set=[]% active set of inequality constraints from pervious iteration
    
    % LQR solution terms ---------------------------------------------------------
    x0 % nominal state (possibly time-varying)
    y0 % nominal output (possibly time-varying)
    u0=zeros(2,1) % nominal input (possibly time-varying)
    R=zeros(2) % input LQR cost
    Qy % output LQR cost
    S % cost-to-go terms: x'Sx + x's1 + s2
    s1
    s2
    Sdot % cost-to-go derivatives
    s1dot
    s2dot
    lqr_is_time_varying % true if TVLQR, false if TILQR

    % whole-body stuff -----------------------------------------------------------
    qtraj % generalize configuration vector or trajectory 
    % note that qtraj can be time-varying, even if LQR system is not
    comtraj % COM state trajectory
    support_times % vector of contact transition times
    supports % (array of) RigidBodySupportState object(s)
    ignore_terrain 
    link_constraints=[] % structure of link motion constraints, see Biped class
    constrained_dofs=[] % array of joint indices
    acceleration_input_frame; % input coordinate frame for desired 
    % generalized accelerations
    plan_shift % linear offset to be applied to x0
    pelvis_z_prev 
    
    % dynamics related -----------------------------------------------------------
    mu % friction coefficient    
  end
  
  methods 
    function obj = QPControllerData(lqr_is_time_varying,data)
      typecheck(lqr_is_time_varying,'logical');
      typecheck(data,'struct');
      data.lqr_is_time_varying = lqr_is_time_varying; % true if LQR problem is tv
      obj = obj@ControllerData(data);
    end
 
    function data=verifyControllerData(~,data)
      assert(isa(data.acceleration_input_frame,'CoordinateFrame'));      
      assert(isa(data.qtraj,'Trajectory') || isnumeric(data.qtraj));      
      if data.lqr_is_time_varying
        if isfield(data,'comtraj')
          assert(isa(data.comtraj,'Trajectory'));
        end
        assert(isa(data.x0,'Trajectory'));
        assert(isa(data.y0,'Trajectory'));
        assert(isa(data.S,'Trajectory') || isnumeric(data.S)); % handle constant case
        assert(isa(data.s1,'Trajectory'));
        assert(isa(data.s2,'Trajectory'));
        if isfield(data,'s1dot')
          assert(isa(data.s1dot,'Trajectory'));
        else
          data.s1dot = fnder(data.s1);
        end
        if isfield(data,'s2dot')
          assert(isa(data.s2dot,'Trajectory'));
        else
          data.s2dot = fnder(data.s2);
        end
        assert(isa(data.u0,'Trajectory'));
      else
        assert(isnumeric(data.y0));
        assert(isnumeric(data.S));
        assert(isnumeric(data.s1));
        assert(isnumeric(data.s2));
        assert(isnumeric(data.u0));
      end
      assert(isnumeric(data.support_times));
      assert(islogical(data.ignore_terrain));
      assert(isnumeric(data.mu));
      assert(isnumeric(data.Qy));
      % optional properties
      if isfield(data,'link_constraints')
        assert(isstruct(data.link_constraints));
      end
      if isfield(data,'R')
        assert(isnumeric(data.R));
      end
      if isfield(data,'A')
        assert(isnumeric(data.A) || isa(data.A,'Trajectory'));
      end
      if isfield(data,'B')
        assert(isnumeric(data.B) || isa(data.B,'Trajectory'));
      end
      if isfield(data,'C')
        assert(isnumeric(data.C) || isa(data.C,'Trajectory'));
      end
      if isfield(data,'D')
        assert(isnumeric(data.D) || isa(data.D,'Trajectory'));
      end
    end
        
    function updateControllerData(obj,data)
      if isfield(data,'lqr_is_time_varying')
        obj.lqr_is_time_varying = data.lqr_is_time_varying;
      end
      if isfield(data,'acceleration_input_frame')
        obj.acceleration_input_frame = data.acceleration_input_frame;
      end
      if isfield(data,'qtraj')
        obj.qtraj = data.qtraj;
      end
      if isfield(data,'comtraj')
        obj.comtraj = data.comtraj;
      end
      if isfield(data,'S')
        obj.S = data.S;
      end
      if isfield(data,'s1')
        obj.s1 = data.s1;
      end
      if isfield(data,'s2')
        obj.s2 = data.s2;
      end
      if isfield(data,'s1dot')
        obj.s1dot = data.s1dot;
      end
      if isfield(data,'s2dot')
        obj.s2dot = data.s2dot;
      end
      if isfield(data,'Qy')
        obj.Qy = data.Qy;       
      end      
      if isfield(data,'y0')
        obj.y0 = data.y0;
      end
      if isfield(data,'x0')
        obj.x0 = data.x0;
      end
      if isfield(data,'u0')
        obj.u0 = data.u0;
      end
      if isfield(data,'mu')
        obj.mu = data.mu;
      end
      if isfield(data,'ignore_terrain')
        obj.ignore_terrain = data.ignore_terrain;
      end
      if isfield(data,'supports')
        obj.supports = data.supports;
      end
      if isfield(data,'support_times')
        obj.support_times = data.support_times;
      end
      if isfield(data,'qp_active_set')
        obj.qp_active_set = data.qp_active_set;
      end
      if isfield(data,'link_constraints')
        obj.link_constraints = data.link_constraints;
      end
      if isfield(data,'constrained_dofs')
        obj.constrained_dofs = data.constrained_dofs;
      end
      if isfield(data,'plan_shift')
        obj.plan_shift = data.plan_shift;
      end
      if isfield(data,'R')
        obj.R = data.R;
      end
      if isfield(data,'A')
        if isa(data.A,'Trajectory')
          obj.A_is_time_varying = true;
        end
        obj.A = data.A;
      end
      if isfield(data,'B')
        if isa(data.B,'Trajectory')
          obj.B_is_time_varying = true;
        end
        obj.B = data.B;
      end
      if isfield(data,'C')
        if isa(data.C,'Trajectory')
          obj.C_is_time_varying = true;
        end
        obj.C = data.C;
      end
      if isfield(data,'D')
        if isa(data.D,'Trajectory')
          obj.D_is_time_varying = true;
        end
        obj.D = data.D;
      end
    end
  end
end