----------------------------
-- FFM LFE5 Top level for apple2fpga
-- http://github.com/emard
----------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.ALL;
use IEEE.numeric_std.all;
-- use IEEE.MATH_REAL.ALL;

library ecp5u;
use ecp5u.components.all;

entity ffmlfe5_apple2 is
port
(
  clk_100mhz_p, clk_100mhz_n: in std_logic;
  -- RS232
  --uart3_txd: out std_logic; -- rs232 txd
  --uart3_rxd: in std_logic; -- rs232 rxd
  -- SD card (SPI)
  sd_f_clk, sd_f_cmd: out std_logic;
  sd_f_d: inout std_logic_vector(3 downto 0);
  sd_f_cdet: in std_logic;
  -- SDRAM
--  dr_clk: out std_logic;
--  dr_cke: out std_logic;
--  dr_cs_n: out std_logic;
--  dr_a: out std_logic_vector(12 downto 0);
--  dr_ba: out std_logic_vector(1 downto 0);
--  dr_ras_n, dr_cas_n: out std_logic;
--  dr_dqm: out std_logic_vector(3 downto 0);
--  dr_d: inout std_logic_vector(31 downto 0);
--  dr_we_n: out std_logic;
  -- FFM Module IO
  fioa: inout std_logic_vector(7 downto 0);
  fiob: inout std_logic_vector(31 downto 20);
  -- Low-Cost HDMI video out
  vid_d_p, vid_d_n: out std_logic_vector(3 downto 0);
  -- ADV7513 video chip
  dv_clk: inout std_logic;
  dv_sda: inout std_logic;
  dv_scl: inout std_logic;
  dv_int: inout std_logic;
  dv_de: inout std_logic;
  dv_hsync: inout std_logic;
  dv_vsync: inout std_logic;
  dv_spdif: inout std_logic;
  dv_mclk: inout std_logic;
  dv_i2s: inout std_logic_vector(3 downto 0);
  dv_sclk: inout std_logic;
  dv_lrclk: inout std_logic;
  dv_d: inout std_logic_vector(23 downto 0)
);
end;

architecture struct of ffmlfe5_apple2 is
  -- keyboard
  alias ps2a_clk : std_logic is fioa(6);
  alias ps2a_data : std_logic is fioa(4);
  -- mouse
  alias ps2b_clk : std_logic is fioa(3);
  alias ps2b_data : std_logic is fioa(1);

  alias audio_l: std_logic is fioa(2);
  alias audio_r: std_logic is fioa(0);

  alias ps2led_green: std_logic is fioa(5); -- green LED
  alias ps2led_red: std_logic is fioa(7); -- red LED

  alias sd_d: std_logic_vector(3 downto 0) is sd_f_d;
  alias sd_clk: std_logic is sd_f_clk;
  alias sd_cmd: std_logic is sd_f_cmd;

  alias n_joy1_up    : std_logic is fiob(20); -- up
  alias n_joy1_down  : std_logic is fiob(21); -- down
  alias n_joy1_left  : std_logic is fiob(22); -- left
  alias n_joy1_right : std_logic is fiob(23); -- right
  alias n_joy1_fire1 : std_logic is fiob(24); -- fire1
  alias n_joy1_fire2 : std_logic is fiob(25); -- fire2

  alias n_joy2_up    : std_logic is fiob(26); -- up
  alias n_joy2_down  : std_logic is fiob(27); -- down
  alias n_joy2_left  : std_logic is fiob(28); -- left 
  alias n_joy2_right : std_logic is fiob(29); -- right  
  alias n_joy2_fire1 : std_logic is fiob(30); -- fire1
  alias n_joy2_fire2 : std_logic is fiob(31); -- fire2 
  
  alias gpdi_dp: std_logic_vector(3 downto 0) is vid_d_p;
  alias gpdi_dn: std_logic_vector(3 downto 0) is vid_d_n;

  signal clk_25MHz: std_logic;

begin
  clksys : entity work.clk_sys_vhdl
  port map
  (
    clkin   => clk_100MHz_p,
    clk_25  => clk_25MHz,
    locked  => open
  );

  E_apple2: entity work.ulx3s_apple2
  port map
  (
    clk_25mhz => clk_25MHz,

    ftdi_txd  => '1',
    wifi_txd  => '1',

    sw => "1111",
    btn => "0000001", -- btn(0) is reset_n

    led(0) => ps2led_red, -- floppy "IN USE>" LED

    audio_l(0) => audio_l,
    audio_r(0) => audio_r,

    -- PS/2 keyboard
    usb_fpga_bd_dp => ps2a_clk,
    usb_fpga_bd_dn => ps2a_data,
    
    gpdi_dp => gpdi_dp,
    gpdi_dn => gpdi_dn,

    sd_d => sd_d,
    sd_clk => sd_clk,
    sd_cmd => sd_cmd
  );
end struct;
