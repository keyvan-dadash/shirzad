function build_all_mex(varargin)
    %----------------------------------------------------------------------
    isDebug = (nargin >= 1) && any(strcmpi(varargin{1}, {'debug','dbg'}));

    if isDebug
        fprintf('*** Building MEX files in DEBUG mode ***\n');
    else
        fprintf('*** Building MEX files in RELEASE mode ***\n');
    end

    thisFile   = mfilename('fullpath');
    projectDir = fileparts(thisFile);

    addpath(projectDir);

    if ispc
        cxxExtra = 'CXXFLAGS="$CXXFLAGS -O3 -march=native -ffast-math -fno-math-errno"';
        if isDebug
            cxxExtra = [cxxExtra ' -g -DDEBUG'];
        end
    else
        % Linux / macOS
        cxxExtra = 'CXXFLAGS=$CXXFLAGS -std=c++17 -O3';
        if isDebug
            cxxExtra = [cxxExtra ' -g -DDEBUG'];
        end
    end

    commonFlags = {
        cxxExtra
    };

    f = @(varargin) fullfile(projectDir, varargin{:});

    targets = {

        struct( ...
            'name',    'mex.qam16_demod_hard_mex', ...
            'source',  f('+mex', 'qam16_demod_hard_mex.cpp'), ...
            'outdir',  f('+mex'), ...
            'extra',   {{}} ...
        )

        struct( ...
            'name',    'mex.qam64_demod_hard_mex', ...
            'source',  f('+mex', 'qam64_demod_hard_mex.cpp'), ...
            'outdir',  f('+mex'), ...
            'extra',   {{}} ...
        )

        struct( ...
            'name',    'mex.qam256_demod_hard_mex', ...
            'source',  f('+mex', 'qam256_demod_hard_mex.cpp'), ...
            'outdir',  f('+mex'), ...
            'extra',   {{}} ...
        )

        struct( ...
            'name',    'mex.payload_worker_mex', ...
            'source',  f('+mex', 'payload_worker_mex.cpp'), ...
            'outdir',  f('+mex'), ...
            'extra',   {{}} ...
        )

        struct( ...
            'name',    'mex.decisionDirectedCarrierSyncMex', ...
            'source',  f('+mex', 'decisionDirectedCarrierSyncMex.cpp'), ...
            'outdir',  f('+mex'), ...
            'extra',   {{}} ...
        )

        struct( ...
            'name',    'mex.schmidlCoxDetectMex', ...
            'source',  f('+mex', 'schmidlCoxDetectMex.cpp'), ...
            'outdir',  f('+mex'), ...
            'extra',   {{}} ...
        )
    };

    cfg = mex.getCompilerConfigurations('C++', 'selected');
    if isempty(cfg)
        error('No MEX C++ compiler configured. Run: mex -setup C++');
    end
    fprintf('Using compiler: %s (%s)\n', cfg.Name, cfg.Location);

    for k = 1:numel(targets)
        t = targets{k};

        fprintf('\n=== Building %s ===\n', t.name);
        if ~isfile(t.source)
            warning('Source not found: %s (skipping)', t.source);
            continue;
        end

        args = [{t.source}, ...
                {'-R2018a'}, ...
                commonFlags, ...
                {'-outdir', t.outdir}, ...
                t.extra];

        fprintf('mex %s\n', strjoin(quote_args(args), ' '));

        try
            mex(args{:});
        catch ME
            fprintf(2, 'Error building %s:\n  %s\n', t.name, ME.message);
            rethrow(ME);
        end
    end

    fprintf('\n*** All MEX builds attempted. ***\n');
end

function c = quote_args(args)
    % For pretty printing the mex command line only.
    c = cell(size(args));
    for i = 1:numel(args)
        a = args{i};
        if contains(a, ' ')
            c{i} = ['"' a '"'];
        else
            c{i} = a;
        end
    end
end
