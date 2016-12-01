%
%    This file was auto-generated using the ACADO Toolkit.
%    
%    While ACADO Toolkit is free software released under the terms of
%    the GNU Lesser General Public License (LGPL), the generated code
%    as such remains the property of the user who used ACADO Toolkit
%    to generate this code. In particular, user dependent data of the code
%    do not inherit the GNU LGPL license. On the other hand, parts of the
%    generated code that are a direct copy of source code from the
%    ACADO Toolkit or the software tools it is based on, remain, as derived
%    work, automatically covered by the LGPL license.
%    
%    ACADO Toolkit is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%    


function make_acado_solver( name )

	% Output file name, and also function name
	if (nargin > 0)
		fileOut = name;
	else
		fileOut = 'acado_solver';
	end;
		
	% Root folder of code generation
	CGRoot = '.';	
	
	% qpOASES embedded source files
	qpOASESSources = [ ...
		'CGRoot/qpoases/SRC/Bounds.cpp ' ...
		'CGRoot/qpoases/SRC/Constraints.cpp ' ...
		'CGRoot/qpoases/SRC/CyclingManager.cpp ' ...
		'CGRoot/qpoases/SRC/Indexlist.cpp ' ...
		'CGRoot/qpoases/SRC/MessageHandling.cpp ' ...
		'CGRoot/qpoases/SRC/QProblem.cpp ' ...
		'CGRoot/qpoases/SRC/QProblemB.cpp ' ...
		'CGRoot/qpoases/SRC/SubjectTo.cpp ' ...
		'CGRoot/qpoases/SRC/Utils.cpp ' ...
		'CGRoot/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp ' ...
		];
		
	% Auto-generated files
	CGSources = [ ...
		'CGRoot/acado_solver_mex.c ' ...
		'CGRoot/acado_solver.c ' ...
		'CGRoot/acado_integrator.c ' ...
		'CGRoot/acado_auxiliary_functions.c ' ...
		'CGRoot/acado_qpoases_interface.cpp ' ...
		];
	if (nargin > 1)
		CGSources = [CGSources extern];
	end
		
	% Adding additional linker flags for Linux
	ldFlags = '';
	if (isunix() && ~ismac())
		ldFlags = '-lrt';
    elseif (ispc)
        ldFlags = '-DWIN32';
	end;

	% Recipe for compilation
	CGRecipe = [ ...
		'mex -O' ...
		' -I. -I''CGRoot'' -I''CGRoot/qpoases'' -I''CGRoot/qpoases/INCLUDE'' -I''CGRoot/qpoases/SRC''' ...
		' ldFlags' ...
		' -D__MATLAB__ -DQPOASES_CUSTOM_INTERFACE="acado_qpoases_interface.hpp" -O CGSources qpOASESSources -output %s.%s' ...
		];

% Compilation
qpOASESSources = regexprep(qpOASESSources, 'CGRoot', CGRoot);
CGSources = regexprep(CGSources, 'CGRoot', CGRoot);

CGRecipe = regexprep(CGRecipe, 'CGRoot', CGRoot);
CGRecipe = regexprep(CGRecipe, 'CGSources', CGSources);
CGRecipe = regexprep(CGRecipe, 'qpOASESSources', qpOASESSources);
CGRecipe = regexprep(CGRecipe, 'ldFlags', ldFlags);

% disp( sprintf( CGRecipe, fileOut, mexext ) ); 
fprintf( 'compiling... ' );
eval( sprintf(CGRecipe, fileOut, mexext) );
fprintf( ['done! --> ' fileOut '.' mexext '\n'] );
