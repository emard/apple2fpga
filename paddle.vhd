-- AUTHOR=EMARD
-- LICENSE=BSD

-- converts digital value to time
-- suitable for PDL signal generation

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity paddle is
  generic
  (
    C_timer_offset : natural := 26; -- adjust to center: input 128 -> ]?PDL(0) -> 128
    C_timer_bits   : natural := 17
  );
  port
  (
    CLK_14M    : in  std_logic;
    PDL0,PDL1,
    PDL2,PDL3  : in  std_logic_vector(7 downto 0); -- 8-bit digital input
    PDL_STROBE : in  std_logic; -- starts counter
    GAMEPORT   : out std_logic_vector(3 downto 0) -- strobe->1->timeout->0
  );
end;

architecture rtl of paddle is
  type T_ipaddle is array(0 to 3) of unsigned(C_timer_bits-2 downto C_timer_bits-10);
  signal S_ipaddle: T_ipaddle;

  type T_paddle is array(0 to 3) of unsigned(C_timer_bits-1 downto 0);
  signal R_paddle: T_paddle;
  
  constant C_pad0: std_logic_vector(C_timer_bits-11 downto 0) := (others => '0');
  
begin
  S_ipaddle(0) <= unsigned("0" & PDL0) + C_timer_offset;
  S_ipaddle(1) <= unsigned("0" & PDL1) + C_timer_offset;
  S_ipaddle(2) <= unsigned("0" & PDL2) + C_timer_offset;
  S_ipaddle(3) <= unsigned("0" & PDL3) + C_timer_offset;
  -- simulate analog conversion of value to time
  G_paddles: for i in 0 to 3 generate
    process(CLK_14M)
    begin
      if rising_edge(CLK_14M) then
        if PDL_STROBE = '1' then
          R_paddle(i)(C_timer_bits-1) <= '1';
          R_paddle(i)(C_timer_bits-2 downto C_timer_bits-10) <= S_ipaddle(i);
          R_paddle(i)(C_timer_bits-11 downto 0) <= (others => '1');
        else
          if R_paddle(i)(C_timer_bits-1) = '1' then
            R_paddle(i) <= R_paddle(i) - 1;
          end if;
        end if;
      end if;
    end process;
    GAMEPORT(i) <= R_paddle(i)(C_timer_bits-1);
  end generate;
end rtl;
