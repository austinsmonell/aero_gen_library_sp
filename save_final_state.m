function save_final_state(IC_name, out)
    IC_ss = out.state.signals.values(end, :)';
    save(strcat('Flight Configurations\IC_Saves\', IC_name), "IC_ss");
end