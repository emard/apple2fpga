--
-- AUTHOR=EMARD
-- LICENSE=BSD
--

-- VHDL Wrapper

LIBRARY ieee;
USE ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;

entity clk_sys_vhdl is
  port
  (
    clkin          : in  std_logic;
    clk_25         : out std_logic;
    locked         : out std_logic
  );
end;

architecture syn of clk_sys_vhdl is
  component clk_sys -- verilog name and its parameters
  port
  (
    clkin          : in  std_logic;
    clk_25         : out std_logic;
    locked         : out std_logic
  );
  end component;

begin
  clk_sys_v_inst: clk_sys
  port map
  (
    clkin          => clkin,
    clk_25         => clk_25,
    locked         => locked
  );
end syn;
