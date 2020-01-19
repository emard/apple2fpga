-- VHDL wrapper for clock

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;

entity clk_25_shift_pixel is
  port
  (
    clki: in std_logic;
    clko,clks1,clks2,locked: out std_logic
  );
end;

architecture syn of clk_25_shift_pixel is
  component clk_25_shift_pixel_v -- verilog name and its parameters
  port
  (
    clki: in std_logic;
    clko,clks1,clks2,locked: out std_logic
  );
  end component;

begin
  clk_25_shift_pixel_v_inst: clk_25_shift_pixel_v
  port map
  (
    clki => clki,
    clko => clko, clks1 => clks1, clks2 => clks2, locked => locked
  );
end syn;
