classdef data_super < handle
  properties(Access = public)
    dfs
    running
    ud
    setup
    acq % function handle to iterate on the data
    cleanup
  end
  methods
    function sup = data_super(dfs, setup, acq, cleanup)
      sup.dfs = dfs;
      sup.setup = setup;
      sup.acq = acq;
      sup.cleanup = cleanup;
      sup.running = false;
      m = sup.dfs.add_menu('uDACS');
      uimenu(m,'Text','Start','callback',{ @sup.set_running, true });
      uimenu(m,'Text','Stop','callback',{ @sup.set_running, false });
      sup.dfs.add_userdata(sup);
    end
    function set_running(sup,~,~,do_start)
      fprintf(1,'set_running()\n');
      if sup.running
        if ~do_start
          sup.running = do_start;
        end
      else
        if do_start
          sup.running = do_start;
          sup.setup(sup);
          while sup.running
            sup.acq(sup);
            pause(1);
          end
          sup.cleanup(sup);
        end
      end
    end
  end
end