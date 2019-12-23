-------------------------------------------------------------------------------
--
-- SPI slave for DISK ][
--
-- AUTHOR=EMARD
-- LICENSE=BSD
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity disk2_spi_slave is
  port
  (
    -- System Interface -------------------------------------------------------
    CLK_14M        : in    std_logic;     -- System clock
    reset          : in    std_logic;
    -- SPI slave Interface ----------------------------------------------------
    CS_N           : in    std_logic;     -- slave chip select
    SCLK           : in    std_logic;     -- SPI clock
    MOSI           : in    std_logic;     -- Data from ESP32 (master out slave in)
    MISO           : inout std_logic;     -- Data to ESP32 (master in slave out)
    -- Track to read, requested by APPLE ][
    track          : in    unsigned(5 downto 0);  -- Track number (0-34)
    -- IRQ for ESP32 to change to new track
    track_change_n : out   std_logic;
    -- Write to Track buffer, APPLE ][ is circular reading this ---------------
    ram_write_addr : out   unsigned(13 downto 0);
    ram_di         : out   unsigned(7 downto 0);
    ram_we         : out   std_logic
  );
end;

architecture rtl of disk2_spi_slave is
  signal R_MISO_shift, R_MOSI_shift, S_MOSI_shift_next: unsigned(7 downto 0);
  signal R_SCLK_shift : std_logic_vector(1 downto 0);
  signal R_track: unsigned(track'range);
  signal R_track_change: std_logic := '0';
  signal R_bit_counter: unsigned(2 downto 0);
  signal R_ram_we: std_logic;
  signal R_ram_di : unsigned(7 downto 0);
  signal R_ram_write_addr: unsigned(ram_write_addr'range);
begin
  -- SPI clock Edge detection shift right and track change detection
  P_SCLK_shift_tracker: process(CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      R_SCLK_shift <= SCLK & R_SCLK_shift(1);
      if track /= R_track then
        R_track_change <= '1';
      else
        R_track_change <= '0';
      end if;
      R_track <= track;
    end if;
  end process P_SCLK_shift_tracker;
  track_change_n <= not R_track_change;

  S_MOSI_shift_next <= R_MOSI_shift(R_MOSI_shift'high-1 downto 0) & MOSI;
  P_SPI_slave: process(CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      if R_track_change = '1' then -- track change has priority
        R_MISO_shift <= "00" & R_track;
        R_bit_counter <= (others => '0');
        R_ram_write_addr <= (others => '1');
        R_ram_we <= '0';
      else
        if R_SCLK_shift = "10" then -- SCLK rising edge
          R_MOSI_shift <= S_MOSI_shift_next;
          R_MISO_shift <= R_MISO_shift(R_MISO_shift'high-1 downto 0) & R_MISO_shift(R_MISO_shift'high);
          R_bit_counter <= R_bit_counter + 1;
          R_ram_we <= '0';
        else
          if R_SCLK_shift = "01" then -- SCLK falling edge
            if R_bit_counter = "000" then
              R_ram_di <= R_MOSI_shift;
              R_ram_write_addr <= R_ram_write_addr + 1;
              R_ram_we <= '1';
            end if;
          else
            R_ram_we <= '0';
          end if;
        end if; -- SCLK rising edge
      end if;
    end if;
  end process P_SPI_slave;
  MISO <= R_MISO_shift(R_MISO_shift'high) when CS_N = '0' else 'Z';
  ram_we <= R_ram_we;
  ram_di <= R_ram_di;
  ram_write_addr <= R_ram_write_addr;
end rtl;
