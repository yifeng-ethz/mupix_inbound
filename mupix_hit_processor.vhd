-- ------------------------------------------------------------------------------------------------------------
-- IP Name:             mupix_hit_processor
-- Author:              Yifeng Wang (yifenwan@phys.ethz.ch)
-- Revision:            1.0
-- Date:                Jun 27, 2025 (file created)
-- Description:         Process the "hit information" from mupix link. The work flow is 
--                      - padding "TS1" into Mu3e Global TS (GTS)
--                      - convert "TS2" into Time-Over-Threshold (ToT)
--                      - convert physical "Col" and "Row" into logical column and row index 
--
--                      This IP aggregates 36 (default) links by selecting only its desired logical channel
-- ------------------------------------------------------------------------------------------------------------
-- ================ synthsizer configuration =================== 	
-- altera vhdl_input_version vhdl_2008 
-- ============================================================= 

-- general 
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.log2;
use ieee.math_real.ceil;
use ieee.std_logic_misc.or_reduce;
use ieee.std_logic_misc.and_reduce;
-- altera-specific
LIBRARY altera_mf;
USE altera_mf.all;
-- get params from hls* macro
@@ set num_mux_ch $max_channel

entity mupix_hit_processor is 
generic (
    N_LINK                  : natural := 36; -- same as the param in Mupix Inbound Parser IP
                                             -- 3 lane(s) * 3 chip(s) * 4 hladder(s) = 36 link(s)
                                             -- demutiplexing will be initated for 36 channels into 1, controlled by "sel" signal through csr
    DEBUG_LV                : natural := 1
);
port (
    @@ for {set i 0} {$i < $num_mux_ch} {incr i} {
    -- +-----------+
    -- | hit_type0 |
    -- +-----------+
    -- the main hit event from mupix physical channel : $i 
    -- max rate = 1/4 
    asi_hit_type0_link${i}_data              : out std_logic_vector(31 downto 0);
    asi_hit_type0_link${i}_valid             : out std_logic;
    asi_hit_type0_link${i}_error             : out std_logic; -- -- error descriptor = {"symbol_error"}
    asi_hit_type0_link${i}_channel           : out std_logic_vector(AVST_CH_WIDTH-1 downto 0); -- logical channel set by csr of Mupix Inbound Parser IP
    @@ }

    -- +-----------------+
    -- | clock and reset |
    -- +-----------------+
    lvdsrxout_clk                   : in  std_logic; -- you may use seperate clock for the data path IPs, but use lvdsrx_outclock is enough
    lvdsrxout_reset                 : in  std_logic
);
end entity mupix_hit_processor;


architecture rtl of mupix_hit_processor is 

begin
    -- ////////////////////////////////////////////////////////////////////////
	-- clock_interface
	-- ////////////////////////////////////////////////////////////////////////
    proc_clock_interface_comb : process (all)
    begin
        rclk            <= lvdsrxout_clk;
        rrst            <= lvdsrxout_reset;
    end process;

    -- ////////////////////////////////////////////////////////////////////////
	-- event_unpacker
	-- ////////////////////////////////////////////////////////////////////////
    proc_event_unpacker : process (rclk)
    begin
        if rising_edge(rclk) then 
            if (rrst = '1') then 

            else 
                -- default
                sel_event_valid         <= '0';
                sel_event_data          <= (others => '0');
                sel_event_channel       <= (others => '0');

                -- input mux
                gen_mux : for i in 0 to N_LINK-1 loop
                    if (to_integer(unsigned(event_channel_link(i))) = to_integer(unsigned(csr.channel_id))) then -- logical link id matches with csr channel id
                        if (event_valid_link(i) = '1') then -- this hit information word is valid
                            if (csr.filter_inerr = '0' or event_error_link(i) = '0') then -- no symbol error
                                sel_event_valid         <= '1';
                                sel_event_data          <= event_data_link(i);
                                sel_event_channel       <= event_channel_link(i);
                            elsif (csr.filter_inerr = '1' and event_error_link(i) = '1') then -- contain symbol error
                                event_unpacker_discard_cnt      <= event_unpacker_discard_cnt + 1;
                            end if;
                        end if;
                    end if;      
                end loop;

            end if;
        end if;

    end process;

    -- ////////////////////////////////////////////////////////////////////////
	-- ts_padder
	-- ////////////////////////////////////////////////////////////////////////
    proc_ts_padder : process (rclk)
    -- pad the ts into global timestamp
    -- be careful: there is possible the ts from event is delayed, so FPGA GTS is ahead. 
    -- use csr.expected_latency to set the grace window to ensure these ts are counted as not overflowed
    -- in other words with upper ts - 1, or with epoch - 1. 
    begin
        if rising_edge(rclk) then 
            if (rrst = '1') then 
            else 
                if (sel_event_valid = '1') then 
                    ts_raw_valid        <= '1';
                    ts_raw              <= sel_event_data(26 downto 16);
                end if;

                if (ts_raw_valid = '1') then 
                    event_gts_valid     <= '1';
                    if (ts_raw > UPPER and ts_raw < counter_gts_8n - csr.expected_latency) then 
                        event_gts       <= counter_gts_8n(47 downto 11) + ts_raw - EPOCH_TIME;
                    else
                        event_gts       <= counter_gts_8n(47 downto 11) + ts_raw;
                    end if;
                end if;

            end if;
        end if;

    end process;

    -- ////////////////////////////////////////////////////////////////////////
	-- gts_counter
	-- ////////////////////////////////////////////////////////////////////////
    proc_gts_counter : process (rclk)
	-- counter of the global timestamp on the FPGA
	-- needs to be 48 bit at 125 MHz
	begin
		if rising_edge(rclk) then
			if (processor_state = RESET and reset_flow = SYNC) then 
				 -- reset counter
				counter_gts_8n		<= (others => '0');
			else
				-- begin counter
				counter_gts_8n		<= counter_gts_8n + to_unsigned(1,counter_gts_8n'length);
			end if;
		end if;
	end process;



end architecture rtl;