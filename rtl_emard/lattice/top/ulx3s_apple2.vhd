-- (c)EMARD
-- License=BSD

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
-- use IEEE.STD_LOGIC_UNSIGNED.ALL;
use ieee.numeric_std.all;

library ecp5u;
use ecp5u.components.all;

entity ulx3s_apple2 is
generic
(
  C_dummy_constant: integer := 0
);
port
(
  clk_25mhz: in std_logic;  -- main clock input from 25MHz clock source

  -- UART0 (FTDI USB slave serial)
  ftdi_rxd: out   std_logic;
  ftdi_txd: in    std_logic;
  -- FTDI additional signaling
  ftdi_ndtr: inout  std_logic;
  ftdi_ndsr: inout  std_logic;
  ftdi_nrts: inout  std_logic;
  ftdi_txden: inout std_logic;

  -- UART1 (WiFi serial)
  wifi_rxd: out   std_logic;
  wifi_txd: in    std_logic;
  -- WiFi additional signaling
  wifi_en: inout  std_logic := 'Z'; -- '0' will disable wifi by default
  wifi_gpio0: inout std_logic;
  wifi_gpio2: inout std_logic;
  wifi_gpio15: inout std_logic;
  wifi_gpio16: inout std_logic;

  -- Onboard blinky
  led: out std_logic_vector(7 downto 0);
  btn: in std_logic_vector(6 downto 0);
  sw: in std_logic_vector(0 to 3);
  oled_csn, oled_clk, oled_mosi, oled_dc, oled_resn: out std_logic;

  -- GPIO (some are shared with wifi and adc)
  gp, gn: inout std_logic_vector(27 downto 0) := (others => 'Z');

  -- FPGA direct USB connector
  usb_fpga_dp, usb_fpga_dn: inout std_logic;
  usb_fpga_pu_dp, usb_fpga_pu_dn: out std_logic;

  -- SHUTDOWN: logic '1' here will shutdown power on PCB >= v1.7.5
  shutdown: out std_logic := '0';

  -- Digital Video (fake differential outputs)
  gpdi_dp, gpdi_dn: out std_logic_vector(3 downto 0);

  -- Flash ROM (SPI0)
  --flash_miso   : in      std_logic;
  --flash_mosi   : out     std_logic;
  --flash_clk    : out     std_logic;
  --flash_csn    : out     std_logic;

  -- SD card (SPI1)
  sd_dat3_csn, sd_cmd_di, sd_dat0_do, sd_dat1_irq, sd_dat2: inout std_logic := 'Z';
  sd_clk: inout std_logic := 'Z';
  sd_cdn, sd_wp: inout std_logic := 'Z'
);
end;

architecture Behavioral of ulx3s_apple2 is
  signal S_reset: std_logic;  
  signal S_oled: std_logic_vector(63 downto 0);
  signal S_enable: std_logic;

  signal ddr_d: std_logic_vector(3 downto 0);
  signal dvid_red, dvid_green, dvid_blue, dvid_clock: std_logic_vector(1 downto 0);
  signal clk_pixel, clk_pixel_shift: std_logic;

  -- APPLE ][

  -- VGA output
  signal
  VGA_CLK,                                            -- Clock
  VGA_HS,                                             -- H_SYNC
  VGA_VS,                                             -- V_SYNC
  VGA_NBLANK,                                         -- not BLANK
  VGA_BLANK,                                          -- BLANK
  VGA_SYNC : std_logic;                               -- SYNC
  signal
  VGA_R,                                              -- Red[9:0]
  VGA_G,                                              -- Green[9:0]
  VGA_B : unsigned(9 downto 0);                       -- Blue[9:0]

  signal clk_140M, clk_28M, clk_14M, clk_2M, PRE_PHASE_ZERO: std_logic;
  signal IO_SELECT, DEVICE_SELECT : std_logic_vector(7 downto 0);
  signal ADDR : unsigned(15 downto 0);
  signal D, PD : unsigned(7 downto 0);

  signal ram_we : std_logic;
  signal VIDEO, HBL, VBL, LD194 : std_logic;
  signal COLOR_LINE : std_logic;
  signal COLOR_LINE_CONTROL : std_logic;
  signal GAMEPORT : std_logic_vector(7 downto 0);
  signal cpu_pc : unsigned(15 downto 0);

  signal K : unsigned(7 downto 0);
  signal read_key : std_logic;

  signal flash_clk : unsigned(22 downto 0) := (others => '0');
  signal power_on_reset : std_logic := '1';
  signal reset : std_logic;

  signal speaker : std_logic;

  signal track : unsigned(5 downto 0);
  signal image : unsigned(9 downto 0);
  signal trackmsb : unsigned(3 downto 0);
  signal D1_ACTIVE, D2_ACTIVE : std_logic;
  signal track_addr : unsigned(13 downto 0);
  signal TRACK_RAM_ADDR : unsigned(13 downto 0);
  signal tra : unsigned(15 downto 0);
  signal TRACK_RAM_DI : unsigned(7 downto 0);
  signal TRACK_RAM_WE : std_logic;

  signal BRAM_WE : std_logic;
  signal BRAM_DQ : std_logic_vector(7 downto 0);
  signal BRAM_ADDR : unsigned(15 downto 0);

begin
  clk_pll: entity work.clk_25_140_28_14
  port map
  (
      CLKI        =>  clk_25mhz,
      CLKOP       =>  clk_140M,  -- 143.75  MHz
      CLKOS       =>  clk_28M,   --  28.75  MHz
      CLKOS2      =>  clk_14M    --  14.375 MHz
  );

  -- TX/RX passthru
  --ftdi_rxd <= wifi_txd;
  --wifi_rxd <= ftdi_txd;

  wifi_en <= '1';
  wifi_gpio0 <= btn(0);
  S_reset <= not btn(0);

  S_enable <= not btn(1); -- btn1 to hold

  oled_inst: entity oled_hex_decoder
  generic map
  (
    C_data_len => S_oled'length
  )
  port map
  (
    clk => clk_14M,
    clken => flash_clk(0),
    en => S_enable,
    data => S_oled,
    spi_resn => oled_resn,
    spi_clk => oled_clk,
    spi_csn => oled_csn,
    spi_dc => oled_dc,
    spi_mosi => oled_mosi
  );
  
  -- APPLE ][

  reset <= (not btn(0)) or power_on_reset;

  power_on : process(CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      if flash_clk(22) = '1' then
        power_on_reset <= '0';
      end if;
    end if;
  end process;

  -- In the Apple ][, this was a 555 timer
  flash_clkgen : process (CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      flash_clk <= flash_clk + 1;
    end if;
  end process;

  -- Paddle buttons
  GAMEPORT <=  "0000" & (not btn(3 downto 1)) & "0";

  COLOR_LINE_CONTROL <= COLOR_LINE and SW(3);  -- Color or B&W mode

  G_apple2: if true generate
  core : entity work.apple2 port map (
    CLK_14M        => CLK_14M,
    CLK_2M         => CLK_2M,
    PRE_PHASE_ZERO => PRE_PHASE_ZERO,
    FLASH_CLK      => flash_clk(22),
    reset          => reset,
    ADDR           => ADDR,
    ram_addr       => BRAM_ADDR(15 downto 0),
    D              => D,
    ram_do         => unsigned(BRAM_DQ(7 downto 0)),
    PD             => PD,
    ram_we         => BRAM_WE,
    VIDEO          => VIDEO,
    COLOR_LINE     => COLOR_LINE,
    HBL            => HBL,
    VBL            => VBL,
    LD194          => LD194,
    K              => K,
    read_key       => read_key,
    AN             => led(7 downto 4),
    GAMEPORT       => GAMEPORT,
    IO_SELECT      => IO_SELECT,
    DEVICE_SELECT  => DEVICE_SELECT,
    pcDebugOut     => cpu_pc,
    speaker        => speaker
    );
  end generate; -- apple2

  vga : entity work.vga_controller port map (
    CLK_28M    => CLK_28M,
    VIDEO      => VIDEO,
    COLOR_LINE => COLOR_LINE_CONTROL,
    HBL        => HBL,
    VBL        => VBL,
    LD194      => LD194,
    VGA_CLK    => VGA_CLK,
    VGA_HS     => VGA_HS,
    VGA_VS     => VGA_VS,
    VGA_BLANK  => VGA_NBLANK,
    VGA_R      => VGA_R,
    VGA_G      => VGA_G,
    VGA_B      => VGA_B
    );
  VGA_SYNC <= '0';

  G_apple2_soc: if false generate
  keyboard : entity work.keyboard port map (
    PS2_Clk  => usb_fpga_dp,
    PS2_Data => usb_fpga_dn,
    CLK_14M  => CLK_14M,
    reset    => reset,
    read     => read_key,
    K        => K
    );
  usb_fpga_pu_dp <= '1';
  usb_fpga_pu_dn <= '1';

  disk : entity work.disk_ii port map (
    CLK_14M        => CLK_14M,
    CLK_2M         => CLK_2M,
    PRE_PHASE_ZERO => PRE_PHASE_ZERO,
    IO_SELECT      => IO_SELECT(6),
    DEVICE_SELECT  => DEVICE_SELECT(6),
    RESET          => reset,
    A              => ADDR,
    D_IN           => D,
    D_OUT          => PD,
    TRACK          => TRACK,
    TRACK_ADDR     => TRACK_ADDR,
    D1_ACTIVE      => D1_ACTIVE,
    D2_ACTIVE      => D2_ACTIVE,
    ram_write_addr => TRACK_RAM_ADDR,
    ram_di         => TRACK_RAM_DI,
    ram_we         => TRACK_RAM_WE
    );

  sdcard_interface : entity work.spi_controller port map (
    CLK_14M        => CLK_14M,
    RESET          => RESET,

    CS_N           => sd_dat3_csn,
    MOSI           => sd_cmd_di,
    MISO           => sd_dat0_do,
    SCLK           => sd_clk,

    track          => TRACK,
    image          => image,

    ram_write_addr => TRACK_RAM_ADDR,
    ram_di         => TRACK_RAM_DI,
    ram_we         => TRACK_RAM_WE
    );
  sd_dat1_irq <= '1';
  sd_dat2     <= '1';
  end generate; -- apple2_soc

  -- selects disk image
  --image <= "0000000" & SW(2 downto 0);
  image <= "000000" & x"0";

  ram_64K: entity work.bram_true2p_1clk
  generic map
  (
        dual_port => false,
        pass_thru_a => false,
        data_width => 8,
        addr_width => 16
  )
  port map
  (
        clk => CLK_14M,
        we_a => bram_we,
        addr_a => std_logic_vector(bram_addr),
        data_in_a => std_logic_vector(d),
        data_out_a => bram_dq
  );

  -- Processor PC on the right four digits
  S_oled(15 downto 0) <= cpu_pc;

  -- Current disk track on middle two digits 
  S_oled(21 downto 16) <= track;

  -- Current disk image on left two digits
  S_oled(31 downto 24) <= image(7 downto 0);
  
  clk_pixel <= CLK_28M;
  clk_pixel_shift <= CLK_140M;
  vga_blank <= not vga_nblank;

  vga2dvi_converter: entity work.vga2dvid
  generic map
  (
      C_ddr     => '1',
      C_depth   => 8 -- 8bpp (8 bit per pixel)
  )
  port map
  (
      clk_pixel => clk_pixel, -- 28 MHz
      clk_shift => clk_pixel_shift, -- 5*28 MHz

      in_red   => std_logic_vector(vga_r(9 downto 2)),
      in_green => std_logic_vector(vga_g(9 downto 2)),
      in_blue  => std_logic_vector(vga_b(9 downto 2)),

      in_hsync => vga_hs,
      in_vsync => vga_vs,
      in_blank => vga_blank,

      -- single-ended output ready for differential buffers
      out_red   => dvid_red,
      out_green => dvid_green,
      out_blue  => dvid_blue,
      out_clock => dvid_clock
  );

  -- vendor specific DDR modules
  -- convert SDR 2-bit input to DDR clocked 1-bit output (single-ended)
  ddr_clock: ODDRX1F port map (D0=>dvid_clock(0), D1=>dvid_clock(1), Q=>ddr_d(3), SCLK=>clk_pixel_shift, RST=>'0');
  ddr_red:   ODDRX1F port map (D0=>dvid_red(0),   D1=>dvid_red(1),   Q=>ddr_d(2), SCLK=>clk_pixel_shift, RST=>'0');
  ddr_green: ODDRX1F port map (D0=>dvid_green(0), D1=>dvid_green(1), Q=>ddr_d(1), SCLK=>clk_pixel_shift, RST=>'0');
  ddr_blue:  ODDRX1F port map (D0=>dvid_blue(0),  D1=>dvid_blue(1),  Q=>ddr_d(0), SCLK=>clk_pixel_shift, RST=>'0');
  -- vendor specific modules for differential output
  gpdi_data_channels: for i in 0 to 3 generate
    gpdi_diff_data: OLVDS port map (A => ddr_d(i), Z => gpdi_dp(i), ZN => gpdi_dn(i));
  end generate;

end Behavioral;
