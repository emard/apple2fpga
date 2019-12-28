-- AUTHOR=EMARD
-- LICENSE=BSD

-- converts digital values to time
-- suitable for PDL signal generation

-- for PDL reading, APPLE ][ set PDL_STROBE pulse shortly to 1
-- during the pulse, this module sets all PDL bits to 1,
-- after the pulse it starts countdown,
-- when counter expires,
-- corresponding PDL bit is set to 0.

-- to show paddle value from applesoft:
-- ]?PDL(0) prints paddle 0
-- ]?PDL(1) prints paddle 1 etc. up to 3

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity paddle is
  generic
  ( -- adjust bits and scale so that
    -- PDL() reading is equal to digital input. example:
    -- ]?PDL(0) reads paddle 0
    C_bits   : natural   := 22;  -- larger ->  larger PDL() reading
    C_scale  : natural   := 106  -- larger -> smaller PDL() reading
  );
  port
  (
    CLK_14M    : in  std_logic;
    -- input 8-bit value from joystick
    PDL0,PDL1,
    PDL2,PDL3  : in  std_logic_vector(7 downto 0); -- 8-bit digital input
    -- APPLE ][ interface
    PDL_STROBE : in  std_logic; -- starts counter
    GAMEPORT   : out std_logic_vector(3 downto 0) -- strobe->1->timeout->0
  );
end;

architecture rtl of paddle is
  signal R_index: unsigned(1 downto 0); -- 4 values for PDL(0) to PDL(3)
  signal i: integer;
  type T_PDL is array(0 to 3) of unsigned(7 downto 0);
  signal S_PDL: T_PDL;
  type T_startval is array(0 to 3) of unsigned(C_bits-2 downto C_bits-11);
  signal R_startval: T_startval;
  type T_paddle is array(0 to 3) of unsigned(C_bits-1 downto 0);
  signal R_paddle: T_paddle;
begin
  S_PDL(0) <= unsigned(PDL0);
  S_PDL(1) <= unsigned(PDL1);
  S_PDL(2) <= unsigned(PDL2);
  S_PDL(3) <= unsigned(PDL3);
  -- simulate analog conversion from value to time
  i <= to_integer(R_index);
  process(CLK_14M)
  begin
      if rising_edge(CLK_14M) then
        R_startval(i) <= ("0" & S_PDL(i) & "0"); -- + C_offset;
        if PDL_STROBE = '1' then
          R_paddle(i)(C_bits-1) <= '1';
          R_paddle(i)(C_bits-2 downto C_bits-11) <= R_startval(i);
          R_paddle(i)(C_bits-12 downto 0) <= (others => '0');
        else
          if R_paddle(i)(C_bits-1) = '1' then
            R_paddle(i) <= R_paddle(i) - C_scale;
          end if;
        end if;
        R_index <= R_index + 1;
      end if;
  end process;
  G_output: for j in 0 to 3 generate
    GAMEPORT(j) <= R_paddle(j)(C_bits-1);
  end generate;
end rtl;
