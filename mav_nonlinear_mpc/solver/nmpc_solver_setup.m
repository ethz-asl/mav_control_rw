clc;
clear all;
close all;

Ts = 0.1; %prediction sampling time
EXPORT = 1;

DifferentialState velocity(3) roll pitch yaw position(3) ;
Control roll_ref pitch_ref thrust;

OnlineData roll_tau;
OnlineData roll_gain;
OnlineData pitch_tau;
OnlineData pitch_gain;
OnlineData linear_drag_coefficient(2);
OnlineData external_forces(3);

n_XD = length(diffStates);
n_U = length(controls);

g = [0;0;9.8066];

%% Differential Equation

R = [cos(yaw)*cos(pitch)  cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw)  (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)); ...
    cos(pitch)*sin(yaw)  cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll)  (sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)); ...
    -sin(pitch)                            cos(pitch)*sin(roll)                            cos(pitch)*cos(roll)];

z_B = R(:,3);

%nonlinear drag model
drag_acc = thrust*[linear_drag_coefficient1 0 0; 0 linear_drag_coefficient2 0; 0 0 0]*R'*velocity;

droll = (1/roll_tau)*(roll_gain*roll_ref - roll);
dpitch = (1/pitch_tau)*(pitch_gain*pitch_ref - pitch);


f = dot([velocity; roll; pitch; yaw; position]) == ...
    [z_B*thrust-g-drag_acc+external_forces;...
    droll; ...
    dpitch;...
    0;...
    velocity;...
    ];

h = [position;...
    velocity;...
    roll;...
    pitch;...
    roll_ref;...
    pitch_ref;...
    z_B(3)*thrust-g(3)];

hN = [position;...
    velocity];

%% MPCexport
acadoSet('problemname', 'mav_position_mpc');

N = 20;
ocp = acado.OCP( 0.0, N*Ts, N );

W_mat = eye(length(h));
WN_mat = eye(length(hN));
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );
ocp.subjectTo(-deg2rad(45) <= [roll_ref; pitch_ref] <= deg2rad(45));
ocp.subjectTo( g(3)/2.0 <= thrust <= g(3)*1.5);
ocp.setModel(f);


mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',        'FULL_CONDENSING_N2'  ); %FULL_CONDENsinG_N2
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',         N                  );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'NO'             	);
mpc.set( 'LEVENBERG_MARQUARDT',          1e-10				);
mpc.set( 'LINEAR_ALGEBRA_SOLVER',        'GAUSS_LU'         );
mpc.set( 'IMPLICIT_INTEGRATOR_NUM_ITS',  2                  );
mpc.set( 'CG_USE_OPENMP',                'YES'              );
mpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES', 'NO'              );
mpc.set( 'CG_USE_VARIABLE_WEIGHTING_MATRIX', 'NO'           );


if EXPORT
    mpc.exportCode('.');
end
