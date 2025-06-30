-- ------------------------------------------------------------------------------------------------------------
-- IP Name:             mupix_inbound_parser
-- Author:              Yifeng Wang (yifenwan@phys.ethz.ch)
-- Revision:            1.0
-- Date:                Jun 2, 2025 (file created)
-- Description:         Parse the incoming data from Mupix fast data lane to get the following 
--                      - lane id[7:0]          : "link identifier", 0xAA/0xBB/0xCC for three lanes or 0xDD for aggregated.
--                                                this is attached as avst channel to every packet generated
--                      - free counter[23:0]    : binary counter, for sanity check, free-running counter after hard reset
--                      - ts counter[7:0]       : gray counter, used for time stamping hit events
--                      - sc data[63:0]         : slow control data word, for the mupix_sc_rx to handle 
--                                                "FLAG"[63:60], "Counter"[59:53]/[empty], "Payload"[54:0]/[59:0]
--                      - event[31:0]           : "Hit Information"
--                                                TS2[4:0], TS1[10:0], Col[6:0], Row[8:0]
--
--                      Each link from Mupix is individually sending all the above data, no lane bonding into link is needed.
--
--                      
--					
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

entity mupix_inbound_parser is 
generic (
    N_LINK                          : natural := 36; -- same as below, the number of mupix links connects to FEB
                                                     -- 3 lane(s) * 3 chip(s) * 4 hladder(s) = 36 link(s)
                                                     -- = max_channel
    CH_ID                           : natural := 0; -- the default channel ID, range from 0 to N_LINK-1, can be configured later on, CH_ID mod 3 will be check against link identifier
    AVST_CH_WIDTH                   : natural := 7;
    DEBUG_LV                        : natural := 1
);
port (
    -- +---------+
    -- | lvds_rx |
    -- +---------+
    asi_lvds_rx_data                : in  std_logic_vector(8 downto 0);
    asi_lvds_rx_error               : in  std_logic_vector(2 downto 0); -- error descriptor = {"loss_sync_pattern" "parity_error" "decode_error"}
    asi_lvds_rx_channel             : in  std_logic_vector(AVST_CH_WIDTH-1 downto 0);

    -- +--------------+
    -- | free_counter |
    -- +--------------+
    -- free counter for sync check
    aso_free_counter_data           : out std_logic_vector(23 downto 0);
    aso_free_counter_valid          : out std_logic;
    aso_free_counter_error          : out std_logic; -- -- error descriptor = {"symbol_error"}
    aso_free_counter_channel        : out std_logic_vector(AVST_CH_WIDTH-1 downto 0);

    -- +------------+
    -- | ts_counter |
    -- +------------+
    -- gray counter, not used
    aso_ts_counter_data             : out std_logic_vector(7 downto 0);
    aso_ts_counter_valid            : out std_logic;
    aso_ts_counter_error            : out std_logic; -- -- error descriptor = {"symbol_error"}
    aso_ts_counter_channel          : out std_logic_vector(AVST_CH_WIDTH-1 downto 0);

    -- +-------+
    -- | sc_rx |
    -- +-------+
    -- the main slow control word from mupix
    aso_sc_rx_data                  : out std_logic_vector(63 downto 0);
    aso_sc_rx_valid                 : out std_logic;
    aso_sc_rx_error                 : out std_logic; -- -- error descriptor = {"symbol_error"}
    aso_sc_rx_channel               : out std_logic_vector(AVST_CH_WIDTH-1 downto 0);

    -- +-----------+
    -- | hit_type0 |
    -- +-----------+
    -- the main hit event from mupix
    -- max rate = 1/4 
    aso_hit_type0_data              : out std_logic_vector(31 downto 0);
    aso_hit_type0_valid             : out std_logic;
    aso_hit_type0_error             : out std_logic; -- -- error descriptor = {"symbol_error"}
    aso_hit_type0_channel           : out std_logic_vector(AVST_CH_WIDTH-1 downto 0);

    -- +-----+
    -- | csr |
    -- +-----+
    -- control and status register slave interface 
    avs_csr_read                    : in  std_logic;
    avs_csr_readdata                : out std_logic_vector(31 downto 0);
    avs_csr_write                   : in  std_logic;
    avs_csr_writedata               : in  std_logic_vector(31 downto 0);
    avs_csr_address                 : in  std_logic_vector(2 downto 0);
    avs_csr_waitrequest             : out std_logic;

    -- +-----------------+
    -- | clock and reset |
    -- +-----------------+
    lvdsrxout_clk                   : in  std_logic;
    lvdsrxout_reset                 : in  std_logic
);
end entity mupix_inbound_parser;


architecture rtl of mupix_inbound_parser is 
    -- 8b10b symbols (ref: https://docs.amd.com/r/en-US/am002-versal-gty-transceivers/8B/10B-Valid-Characters)
    -- =======================================================================
    -- code name            Bits HGF EDCBA          symbol RD- abcdei fghj
    -- =======================================================================
    -- K28.5                101 11100               001111 1010
    -- K28.2                010 11100               001111 0101
    -- K28.0                000 11100               001111 0100
    constant K28_5                  : std_logic_vector(7 downto 0) := "10111100"; -- 0xBC
    constant K28_2                  : std_logic_vector(7 downto 0) := "01011100"; -- 0x5C
    constant K28_0                  : std_logic_vector(7 downto 0) := "00011100"; -- 0x1C

    -- ------------------------------------------------------------------------
	-- clock interface
	-- ------------------------------------------------------------------------
    signal rclk                     : std_logic;
    signal rrst                     : std_logic;

    -- ------------------------------------------------------------------------
    -- csr_hub 
    -- ------------------------------------------------------------------------
    type csr_t is record
        go                      : std_logic;
        channel_id              : std_logic_vector(AVST_CH_WIDTH-1 downto 0);
        struc_err_counter       : std_logic_vector(31 downto 0);
    end record;
    constant CSR_DEFAULT        : csr_t := (
        go                      => '1',
        channel_id              => std_logic_vector(to_unsigned(CH_ID,AVST_CH_WIDTH)),
        struc_err_counter       => std_logic_vector(to_unsigned(0,AVST_CH_WIDTH))
    );
    signal csr                  : csr_t := CSR_DEFAULT;

    -- ------------------------------------------------------------------------
    -- store_rx2egress
    -- ------------------------------------------------------------------------
    type egress_t is record
        linkID                      : std_logic_vector(3 downto 0);
        linkID_valid                : std_logic;
        linkID_err                  : std_logic;
        free_counter                : std_logic_vector(23 downto 0);
        free_counter_err            : std_logic;
        free_counter_valid          : std_logic;
        ts_counter_rx               : std_logic_vector(7 downto 0);
        ts_counter_err              : std_logic;
        ts_counter_valid            : std_logic;
        sc_data                     : std_logic_vector(63 downto 0);
        sc_data_err                 : std_logic;
        sc_data_valid               : std_logic;
        hit_info                    : std_logic_vector(31 downto 0);
        hit_info_err                : std_logic;
        hit_info_valid              : std_logic;
    end record;
    constant EGRESS_DEFAULT         : egress_t := (
        linkID                      => std_logic_vector(to_unsigned(0,4)),
        linkID_valid                => '0',
        linkID_err                  => '0',
        free_counter                => std_logic_vector(to_unsigned(0,24)),
        free_counter_err            => '0',
        free_counter_valid          => '0',
        ts_counter_rx               => std_logic_vector(to_unsigned(0,8)),
        ts_counter_err              => '0',
        ts_counter_valid            => '0',
        sc_data                     => std_logic_vector(to_unsigned(0,64)),
        sc_data_err                 => '0',
        sc_data_valid               => '0',
        hit_info                    => std_logic_vector(to_unsigned(0,32)),
        hit_info_err                => '0',
        hit_info_valid              => '0'
    );
    signal egress                   : egress_t := EGRESS_DEFAULT;

    -- ------------------------------------------------------------------------
    -- parser
    -- ------------------------------------------------------------------------
    type parse_state_t is (MASK, IDLE, RX_LINK_ID, RX_SC, RX_COUNTERS, RX_HIT, LOG_ERROR);
    signal parse_state                      : parser_state_t;
    type LDCol2_flow_t is (S1, S2, S3, ERROR);
    signal LDCol2_flow                      : LDCol2_flow_t;
    type LDCol1_flow_t is (S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, ERROR);
    signal LDCol1_flow                      : LDCol1_flow_t;
    type LDPix_flow_t is (S0, S1, S2, S3, ERROR);
    signal LDPix_flow                       : LDPix_flow_t;
    type RDcol_flow_t is (S0, S1, S2, S3, ERROR);
    signal RDcol_flow                       : RDcol_flow_t;

    type debug_msg_t is record 
        parser_struc_err_cnt                : unsigned(31 downto 0);
    end record;
    constant DEBUG_MSG_DEFAULT              : debug_msg_t := (
        parser_struc_err_cnt                => std_logic_vector(to_unsigned(0,32))
    );
    signal debug_msg                        : debug_msg_t := DEBUG_MSG_DEFAULT;

    signal linkID_rx_valid                  : std_logic;

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
	-- csr_hub
	-- ////////////////////////////////////////////////////////////////////////
    proc_csr_hub : process (rclk)
    begin
        if (rising_edge(rclk)) then 
            if (rrst = '1') then 
                csr             <= CSR_DEFAULT;
            else
                -- default 
                avs_csr_waitrequest         <= '1';
                avs_csr_readdata            <= (others => '0');

                if (avs_csr_read = '1') then 
                    -- default
                    avs_csr_waitrequest         <= '0';
                    case to_integer(unsigned(avs_csr_address)) is 
                        when 0 =>
                            avs_csr_readdata(0)         <= csr.go;
                        when 1 => -- link id (physical) to channel id (logical) mapping
                            avs_csr_readdata(AVST_CH_WIDTH-1 downto 0)         <= csr.channel_id;
                        when 2 => -- structural error counter
                            avs_csr_readdata            <= csr.struc_err_counter;
                        when others =>
                            null;
                    end case;

                elsif (avs_csr_write = '1') then 
                    -- default 
                    avs_csr_waitrequest         <= '0';
                    case to_integer(unsigned(avs_csr_address)) is 
                        when 0 =>
                            csr.go              <= avs_csr_writedata(0);
                        when 1 => 
                            csr.channel_id      <= avs_csr_writedata(AVST_CH_WIDTH-1 downto 0);
                        when 2 => 
                            -- TODO: reset on request?
                        when others =>
                            null;
                    end case;

                else 
                    -- routine 
                    csr.struc_err_counter           <= std_logic_vector(debug_msg.parser_struc_err_cnt); -- update counter

                end if;
            end if;
        end if;
    end process;

    -- ////////////////////////////////////////////////////////////////////////
    -- store_rx2egress
    -- ////////////////////////////////////////////////////////////////////////
    proc_store_rx2egress : process (rclk)
    begin
        if (rising_edge(rclk)) then 
            -- ------------------------------------------------------
            -- latch register from parser process
            -- ------------------------------------------------------
            if (rrst = '1') then 
                egress          <= EGRESS_DEFAULT;
            else
                -- note: do not store on structural error, only store and mark with symbol error
                -- link id (4 bits)
                egress.linkID_valid <= '0';
                if (linkID_rx_valid = '1') then 
                    egress.linkID       <= linkID_rx;
                    egress.linkID_err   <= rx_link_id_symbol_err;
                    egress.linkID_valid <= '1';
                end if;
            
                -- free counter
                egress.free_counter_valid   <= '0';
                if (free_counter_rx_valid = '1') then 
                    egress.free_counter         <= free_counter_rx;
                    egress.free_counter_err     <= rx_counter_symbol_err;
                    egress.free_counter_valid   <= '1';
                end if;

                -- ts counter
                egress.ts_counter_valid     <= '0'; 
                if (ts_counter_rx_valid = '1') then 
                    egress.ts_counter           <= ts_counter_rx;
                    egress.ts_counter_err       <= rx_counter_symbol_err;
                    egress.ts_counter_valid     <= '1';
                end if;

                -- sc data 
                egress.sc_data_valid        <= '0';
                if (sc_data_rx_valid = '1') then 
                    egress.sc_data          <= sc_data_rx;
                    egress.sc_data_err      <= sc_data_rx_symbol_err;
                    egress.sc_data_valid    <= '1';
                end if;
                
                -- hit info
                egress.hit_info_valid       <= '0';
                if (hit_info_rx_valid = '1') then 
                    egress.hit_info(31 downto 27)   <= hit_info_ts2(4 downto 0);
                    egress.hit_info(26 downto 16)   <= hit_info_ts1(10 downto 0);
                    egress.hit_info(15 downto 9)    <= hit_info_col(6 downto 0);
                    egress.hit_info(8 downto 0)     <= hit_info_row(8 downto 0);
                    egress.hit_info_valid           <= '1';
                    egress.hit_info_err             <= hit_info_symbol_err;
                end if;
            end if;

            -- ------------------------------------------------------
            -- connect egress registers to output port (reg out)
            -- ------------------------------------------------------
            -- free counter
            aso_free_counter_valid      <= egress.free_counter_valid;
            aso_free_counter_data       <= egress.free_counter;
            aso_free_counter_error      <= egress.free_counter_err;
            aso_free_counter_channel    <= csr.channel_id;

            -- ts counter
            aso_ts_counter_valid        <= egress.ts_counter_valid;
            aso_ts_counter_data         <= egress.ts_counter;
            aso_ts_counter_error        <= egress.ts_counter_err;
            aso_ts_counter_channel      <= csr.channel_id;

            -- sc data 
            aso_sc_rx_valid             <= egress.sc_data_valid;
            aso_sc_rx_data              <= egress.sc_data;
            aso_sc_rx_error             <= egress.sc_data_err;
            aso_sc_rx_channel           <= csr.channel_id;

            -- hit info 
            aso_hit_type0_valid         <= egress.hit_info_valid;
            aso_hit_type0_data          <= egress.hit_info;
            aso_hit_type0_error         <= egress.hit_info_err;
            aso_hit_type0_channel       <= csr.channel_id;
            

            

        end if;
    end process;



    -- ////////////////////////////////////////////////////////////////////////
    -- parser
    -- ////////////////////////////////////////////////////////////////////////
    proc_parser : process (rclk)
    begin
        if (rising_edge(rclk)) then 
            if (rrst = '1') then 
                parser_state                    <= MASK;
                debug_msg.parser_struc_err_cnt  <= (others => '0');

            else 
                -- default
                linkID_rx_valid         <= '0';
                free_counter_rx_valid   <= '0';
                ts_counter_rx_valid     <= '0';
                sc_data_rx_valid                <= '0';
                hit_info_rx_valid               <= '0';
                
                case parser_state is 
                    when MASK => 
                        if (and_reduce(asi_lvds_rx_error) = '0' and csr.go = '1') then -- mupix lane is error free from lvds rx point of view
                            parser_state        <= IDLE;
                        end if;

                    when IDLE => 
                        -- trigger on K28.2 -> LDCol1 - SC word incoming in the next word
                        if (asi_lvds_rx_data(8) = '1' and asi_lvds_rx_data(7 downto 0) = K28_2) then 
                            parser_state            <= RX_SC; 
                        end if;
                        -- trigger by K28.0 -> LDCol2 - link id already in this word
                        if (asi_lvds_rx_data(8) = '1' and asi_lvds_rx_data(7 downto 0) = K28_0) then 
                            parser_state            <= RX_LINK_ID;
                            LDCol2_flow             <= S1;
                        end if;
                        -- block on 8b10b error
                        if (or_reduce(asi_lvds_rx_error)(1 downto 0) = '1') then 
                            parser_state            <= IDLE;
                        end if;
                        -- do not start issued by csr / mask on any lane corrupt error 
                        if (csr.go = '0' or asi_lvds_rx_error(2) = '1') then 
                            parser_state            <= MASK;
                        end if;

                    when RX_LINK_ID =>
                        -- on 8b10b symbol error - mark it but continue with this word, so we do not lose word boundary
                        if (or_reduce(asi_lvds_rx_error(1 downto 0)) = '1') then 
                            rx_link_id_symbol_err           <= '1'; -- symbol error during decoding
                        end if;
                        -- on structural error 
                        LDCol2_flow             <= ERROR; -- break the flow
                        -- ------------------------------------------------------------------------------------------
                        -- Flow to detect MuPix LDCol2 state: 
                        -- > @capture   <lane identifier> - "link identifier", 0xAA/0xBB/0xCC for three lanes or 0xDD for aggregated 
                        -- >                              -> linkID_rx
                        -- > @return    <error flag> - rx_link_id_symbol_err
                        -- ------------------------------------------------------------------------------------------
                        case LDCol2_flow is -- only preceed when structually correct 
                            when S1 => -- "Fixed Pattern"
                                if (asi_lvds_rx_data(8) = '0') then 
                                    if (asi_lvds_rx_data(3 downto 0) = asi_lvds_rx_data(7 downto 4)) then -- further check 0xXY repeated pattern, X=Y
                                        LDCol2_flow             <= S2;
                                        linkID_rx               <= asi_lvds_rx_data(3 downto 0);
                                    end if;
                                end if;
                            when S2 => -- "K28.0"
                                -- continue to check K28.0 symbol
                                if (asi_lvds_rx_data(8) = '1' and asi_lvds_rx_data(7 downto 0) = K28_0) then 
                                    LDCol2_flow             <= S3;
                                end if;
                            when S3 => -- [normal exit] "Fixed Pattern" 
                                rx_link_id_symbol_err            <= '0';
                                if (asi_lvds_rx_data(8) = '0') then 
                                    if (asi_lvds_rx_data(3 downto 0) = asi_lvds_rx_data(7 downto 4) and asi_lvds_rx_data(3 downto 0) = linkID_rx) then -- confirm link id twice
                                        parser_state            <= RX_COUNTERS; -- [=>] parser state 
                                        LDCol2_flow             <= S0; -- reset this flow
                                        LDPix_flow              <= S0; -- entry point of that flow (maybe unnecessary)
                                        linkID_rx_valid         <= '1';
                                    end if;
                                end if;
                            when ERROR => -- [error exit]
                                parser_state            <= LOG_ERROR; -- [=>] parser state 
                                LDCol2_flow             <= S0; -- reset this flow                                   
                                rx_link_id_symbol_err    <= '0';
                            when others =>
                                null;
                        end case;

                    when RX_SC => 
                        -- on 8b10b symbol error - mark it but continue with this word, so we do not lose word boundary
                        if (or_reduce(asi_lvds_rx_error(1 downto 0)) = '1') then 
                            sc_data_rx_symbol_err           <= '1'; -- symbol error during decoding
                        end if;
                        -- on structural error
                        LDCol1_flow                  <= ERROR; -- enter break state
                        case LDCol1_flow is 
                            when S1 => -- "K28.5"
                                if (asi_lvds_rx_data(8) = '1' and asi_lvds_rx_data(7 downto 0) = K28_5) then 
                                    LDCol1_flow         <= S2;
                                end if;
                            when S2 => -- "K28.5"
                                if (asi_lvds_rx_data(8) = '1' and asi_lvds_rx_data(7 downto 0) = K28_2) then 
                                    LDCol1_flow         <= S3;
                                end if;
                            when S3 => -- "K28.5"
                                if (asi_lvds_rx_data(8) = '1' and asi_lvds_rx_data(7 downto 0) = K28_5) then 
                                    LDCol1_flow         <= S4;
                                end if;
                            when S4 => -- "SC_data[31:24]"
                                if (asi_lvds_rx_data(8) = '0') then 
                                    LDCol1_flow                     <= S5;
                                    sc_data_rx(31 downto 24)        <= asi_lvds_rx_data(7 downto 0);
                                end if;
                            when S5 => -- "SC_data[23:16]"
                                if (asi_lvds_rx_data(8) = '0') then 
                                    LDCol1_flow                     <= S6;
                                    sc_data_rx(23 downto 16)        <= asi_lvds_rx_data(7 downto 0);
                                end if;
                            when S6 => -- "SC_data[15:8]"
                                if (asi_lvds_rx_data(8) = '0') then 
                                    LDCol1_flow                     <= S7;
                                    sc_data_rx(15 downto 8)         <= asi_lvds_rx_data(7 downto 0);
                                end if;
                            when S7 => -- "SC_data[7:0]"
                                if (asi_lvds_rx_data(8) = '0') then 
                                    LDCol1_flow                     <= S8;
                                    sc_data_rx(7 downto 0)          <= asi_lvds_rx_data(7 downto 0);
                                end if;
                            when S8 => -- "SC_data[63:56]"
                                if (asi_lvds_rx_data(8) = '0') then 
                                    LDCol1_flow                     <= S9;
                                    sc_data_rx(63 downto 56)        <= asi_lvds_rx_data(7 downto 0);
                                end if;
                            when S9 => -- "SC_data[53:48]"
                                if (asi_lvds_rx_data(8) = '0') then 
                                    LDCol1_flow                     <= S10;
                                    sc_data_rx(53 downto 48)        <= asi_lvds_rx_data(7 downto 0);
                                end if;
                            when S10 => -- "SC_data[47:40]"
                                if (asi_lvds_rx_data(8) = '0') then 
                                    LDCol1_flow                     <= S11;
                                    sc_data_rx(47 downto 40)        <= asi_lvds_rx_data(7 downto 0);
                                end if;
                            when S11 => -- "SC_data[39:32]" / [normal exit]
                                if (asi_lvds_rx_data(8) = '0') then 
                                    LDCol1_flow                     <= S1; -- reset this flow
                                    parser_state                    <= IDLE;
                                    sc_data_rx_valid                <= '1';
                                    sc_data_rx(39 downto 32)        <= asi_lvds_rx_data(7 downto 0);
                                    sc_data_rx_symbol_err           <= '0';
                                end if;
                            when ERROR => -- [error exit]
                                parser_state                    <= LOG_ERROR;
                                LDCol1_flow                     <= S1;
                                sc_data_rx_symbol_err           <= '0';
                            when others =>
                        end case;

                    when RX_COUNTERS => 
                        -- on 8b10b symbol error - mark it but continue with this word, so we do not lose word boundary
                        if (or_reduce(asi_lvds_rx_error(1 downto 0)) = '1') then 
                            rx_counter_symbol_err            <= '1'; -- symbol error during decoding
                        end if;
                        -- on structural error
                        LDPix_flow                  <= ERROR; -- enter break state 
                        -- ------------------------------------------------------------------------------------------
                        -- Flow to capture LDPix
                        -- > @capture   <free counter[23:0]> - binary counter, for sanity check, free-running counter after hard reset
                        -- >            <ts counter[7:0]> - gray counter, used for time stamping hit events
                        -- > @return    <error flag> - rx_counter_symbol_err
                        -- ------------------------------------------------------------------------------------------
                        case LDPix_flow is -- only preceed when structually correct 
                            when S0 => -- "Counter[23:16]"
                                if (asi_lvds_rx_data(8) = '0') then 
                                    LDPix_flow                      <= S1;
                                    free_counter_rx(23 downto 16)   <= asi_lvds_rx_data(7 downto 0);
                                end if;
                            when S1 => -- "Counter[15:8]"
                                if (asi_lvds_rx_data(8) = '0') then 
                                    LDPix_flow                      <= S2;
                                    free_counter_rx(15 downto 8)    <= asi_lvds_rx_data(7 downto 0);
                                end if;
                            when S2 => -- "Counter[7:0]"
                                if (asi_lvds_rx_data(8) = '0') then 
                                    LDPix_flow                      <= S3;
                                    free_counter_rx(7 downto 0)     <= asi_lvds_rx_data(7 downto 0);
                                    free_counter_rx_valid           <= '1';
                                end if;
                            when S3 => -- "TSCounter[7:0]"
                                if (asi_lvds_rx_data(8) = '0') then -- prepare to exit, end of the word
                                    LDPix_flow                  <= S4;
                                    ts_counter_rx(7 downto 0)      <= asi_lvds_rx_data(7 downto 0);
                                    free_counter_rx_valid       <= '0';
                                    ts_counter_rx_valid         <= '1';
                                end if;
                            when S4 => -- [normal exit]
                                rx_counter_symbol_err           <= '0';
                                if (asi_lvds_rx_data(8) = '1' and asi_lvds_rx_data(7 downto 0) = K28_5) then -- go to Fillber
                                    parser_state                <= IDLE; -- [=>] PD
                                    LDPix_flow                  <= S0;
                                elsif (asi_lvds_rx_data(8) = '1' and asi_lvds_rx_data(7 downto 0) = K28_2) then -- go to SC
                                    parser_state                <= RX_SC; -- [=>] PD
                                    LDPix_flow                  <= S0;
                                elsif (asi_lvds_rx_data(8) = '0') then -- go to hit 
                                    parser_state                <= RX_HIT; -- [=>] RDCol
                                    RDcol_flow                  <= S1; -- first time entry point. ex: S1-S3|S0-S3|S0-S3|...
                                    LDPix_flow                  <= S0; -- reset, not necessary?
                                    hit_info_ts2(4 downto 0)    <= asi_lvds_rx_data(7 downto 3);
                                    hit_info_ts1(10 downto 8)   <= asi_lvds_rx_data(2 downto 0);
                                    if (or_reduce(asi_lvds_rx_error(1 downto 0)) = '1') then
                                        hit_info_symbol_err         <= '1';
                                    end if;
                                end if;   
                            when ERROR => -- [error exit]
                                parser_state                    <= LOG_ERROR;
                                LDPix_flow                      <= S0;
                                rx_counter_symbol_err           <= '0';
                            when others =>
                                null;
                        end case;

                    when RX_HIT =>
                        -- on structural error
                        RDcol_flow                  <= ERROR; -- enter break state 
                        case RDcol_flow is 
                            when S0 => -- [normal exit or loop]
                                hit_info_symbol_err             <= '0';
                                if (asi_lvds_rx_data(8) = '0') then
                                    RDcol_flow                  <= S1;
                                    hit_info_ts2(4 downto 0)    <= asi_lvds_rx_data(7 downto 3);
                                    hit_info_ts1(10 downto 8)   <= asi_lvds_rx_data(2 downto 0);
                                elsif (asi_lvds_rx_data(8) = '1' and asi_lvds_rx_data(7 downto 0) = K28_2) then
                                    parser_state                <= RX_SC; 
                                    RDcol_flow                  <= S1; -- reset? 
                                elsif (asi_lvds_rx_data(8) = '1' and asi_lvds_rx_data(7 downto 0) = K28_5) then
                                    parser_state                <= IDLE;
                                    RDcol_flow                  <= S1; -- reset? 
                                else 
                                    RDcol_flow                  <= ERROR;
                                end if;
                            when S1 => 
                                if (asi_lvds_rx_data(8) = '0') then
                                    RDcol_flow                  <= S2;
                                    hit_info_ts1(7 downto 0)    <= asi_lvds_rx_data(7 downto 0);
                                end if;
                            when S2 =>
                                if (asi_lvds_rx_data(8) = '0') then
                                    RDcol_flow                  <= S3;
                                    hit_info_col(6 downto 0)    <= asi_lvds_rx_data(7 downto 1);
                                    hit_info_row(8)             <= asi_lvds_rx_data(0);
                                end if;
                            when S3 => -- [go back to S0]
                                if (asi_lvds_rx_data(8) = '0') then
                                    RDcol_flow                  <= S0;
                                    hit_info_rx_valid           <= '1';
                                    hit_info_row(7 downto 0)    <= asi_lvds_rx_data(7 downto 0);
                                end if;
                            when ERROR => -- [error exit]
                                RDcol_flow                      <= S1;
                                parser_state                    <= LOG_ERROR;
                                hit_info_symbol_err             <= '0';
                            when others =>
                        end case;
                        -- on 8b10b symbol error - mark it but continue with this word, so we do not lose word boundary
                        if (or_reduce(asi_lvds_rx_error(1 downto 0)) = '1' and RDcol_flow /= ERROR) then 
                            hit_info_symbol_err                 <= '1'; -- symbol error during decoding
                        end if;

                    when LOG_ERROR => 
                        parser_state                    <= MASK;
                        linkID_rx_err                   <= '0'; -- reset this 
                        debug_msg.parser_struc_err_cnt  <= debug_msg.parser_struc_err_cnt + 1; 
                    when others =>
                        null;
                end case;

            end if;
        end if;

    end process;


    
end architecture rtl;