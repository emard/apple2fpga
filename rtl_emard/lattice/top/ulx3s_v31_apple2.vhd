-- (c)EMARD
-- License=BSD

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

-- package for usb joystick report decoded structure
use work.report_decoded_pack.all;

library ecp5u;
use ecp5u.components.all;

entity ulx3s_v31_apple2 is
generic
(
  C_oled        : boolean := false; -- OLED display HEX debug
  -- PS/2 keyboard at (enable one of):
  C_kbd_us2     : boolean := true;  -- onboard micro USB with OTG adapter
  C_kbd_us3     : boolean := false; -- PMOD US3 at GP,GN 25,22,21
  C_kbd_us4     : boolean := false; -- PMOD US4 at GP,GN 24,23,20
  C_kbd_esp32   : boolean := false; -- ESP32->PS2 (wifi_gpio22=clk, wifi_gpio21=data)
  -- USB joystick at (enable one of):
  C_joy_us2     : boolean := false; -- onboard micro USB with OTG adapter
  C_joy_us3     : boolean := false; -- PMOD US3 at GP,GN 25,22,21
  C_joy_us4     : boolean := false;  -- PMOD US4 at GP,GN 24,23,20
  -- apple ][ disk
  C_apple2_disk : boolean := true;  -- false BTNs debug to select track
  -- C_apple2_disk = true, then enable one of
  C_sdcard      : boolean := false; -- NIB images written to raw SD card
  C_esp32       : boolean := true   -- ESP32 disk2.py micropython DISK ][ server
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
  wifi_en: inout  std_logic; -- '0' will disable wifi by default
  wifi_gpio0: inout std_logic; -- spi slave request irq
  wifi_gpio19: inout std_logic; -- spi slave en
  wifi_gpio21: inout std_logic; -- ps2 kbd data
  wifi_gpio22: inout std_logic; -- ps2 kbd clk
  wifi_gpio26: inout std_logic; -- spi slave clk
  wifi_gpio27: inout std_logic;
  --wifi_gpio35: inout std_logic;

  -- Onboard blinky
  led: out std_logic_vector(7 downto 0);
  btn: in std_logic_vector(6 downto 0);
  sw: in std_logic_vector(0 to 3);
  oled_csn, oled_clk, oled_mosi, oled_dc, oled_resn: out std_logic;

  -- Audio
  audio_l, audio_r, audio_v: out std_logic_vector(3 downto 0);

  -- GPIO (some are shared with wifi and adc)
  gp, gn: inout std_logic_vector(27 downto 0);

  -- FPGA direct USB connector
  usb_fpga_dp, usb_fpga_dn: in std_logic;
  usb_fpga_bd_dp, usb_fpga_bd_dn: inout std_logic;
  usb_fpga_pu_dp, usb_fpga_pu_dn: out std_logic;

  -- SHUTDOWN: logic '1' here will shutdown power on PCB >= v1.7.5
  shutdown: out std_logic := '0';

  -- Digital Video (fake differential outputs)
  gpdi_dp: out std_logic_vector(3 downto 0);

  -- Flash ROM
  --flash_miso   : in      std_logic;
  --flash_mosi   : out     std_logic;
  --flash_clk    : out     std_logic;
  --flash_csn    : out     std_logic;

  -- SD card
  sd_d: inout std_logic_vector(3 downto 0);
  sd_clk, sd_cmd: inout std_logic
);
end;

architecture Behavioral of ulx3s_v31_apple2 is
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

  -- invert active LOW sync
  signal vga_hsync, vga_vsync: std_logic;
  -- after OSD module 
  signal osd_vga_r, osd_vga_g, osd_vga_b: std_logic_vector(7 downto 0);
  signal osd_vga_hsync, osd_vga_vsync, osd_vga_blank: std_logic;
  -- invert CS to get CSN
  signal spi_csn: std_logic;

  signal clk_140M, clk_28M, clk_14M, clk_2M, PRE_PHASE_ZERO: std_logic;
  signal IO_SELECT, DEVICE_SELECT : std_logic_vector(7 downto 0);
  signal ADDR : unsigned(15 downto 0);
  signal D, PD : unsigned(7 downto 0);

  signal ram_we : std_logic;
  signal VIDEO, HBL, VBL, LD194 : std_logic;
  signal COLOR_LINE : std_logic;
  signal COLOR_LINE_CONTROL : std_logic;
  signal PDL_STROBE : std_logic;
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
  signal D1_ACTIVE, D2_ACTIVE : std_logic;
  signal track_addr : unsigned(13 downto 0);
  signal TRACK_RAM_ADDR : unsigned(13 downto 0);
  signal tra : unsigned(15 downto 0);
  signal TRACK_RAM_DI : unsigned(7 downto 0);
  signal TRACK_RAM_WE : std_logic;

  signal BRAM_WE : std_logic;
  signal BRAM_DQ : std_logic_vector(7 downto 0);
  signal BRAM_ADDR : unsigned(15 downto 0);
  
  -- USB
  signal clk_usb, clk_6M: std_logic;
  signal S_hid_report: std_logic_vector(63 downto 0);
  signal S_hid_report_valid: std_logic;
  signal S_hid_report_decoded: T_report_decoded;

  -- PMOD with US3 and US4
  -- ULX3S direct or pins down and flat cable: don't swap GP/GN, normal differential input
  -- ULX3S pins up and flat cable: swap GP/GN and invert differential input

  alias us3_fpga_bd_dp: std_logic is gp(25);
  alias us3_fpga_bd_dn: std_logic is gn(25);

  alias us3_fpga_pu_dp: std_logic is gp(22);
  alias us3_fpga_pu_dn: std_logic is gn(22);

  --alias us3_fpga_n_dp: std_logic is gp(21); -- pins up flat cable
  --signal us3_fpga_dp: std_logic; -- pins up flat cable
  alias us3_fpga_dp: std_logic is gp(21); -- direct

  alias us4_fpga_bd_dp: std_logic is gp(24);
  alias us4_fpga_bd_dn: std_logic is gn(24);

  alias us4_fpga_pu_dp: std_logic is gp(23);
  alias us4_fpga_pu_dn: std_logic is gn(23);

  --alias us4_fpga_n_dp: std_logic is gp(20); -- pins up flat cable
  --signal us4_fpga_dp: std_logic; -- pins up flat cable
  alias us4_fpga_dp: std_logic is gp(20); -- direct

begin
  -- 60Hz frame rate
  clk_apple2: entity work.clk_25_140_28_14
  port map
  (
      CLKI        =>  clk_25mhz,
      CLKOP       =>  clk_140M,  -- 143.75  MHz
      CLKOS       =>  clk_28M,   --  28.75  MHz
      CLKOS2      =>  clk_14M    --  14.375 MHz
  );

  -- 56Hz frame rate
  --clk_apple2: entity work.clk_25_shift_pixel
  --port map
  --(
  --    clki        =>  clk_25mhz,
  --    clko        =>  clk_140M,  -- 135.00  MHz
  --    clks1       =>  clk_28M,   --  27.00  MHz
  --    clks2       =>  clk_14M    --  13.50  MHz
  --);
  
  clk_usb: entity work.clk_25_125_68_6_25
  port map
  (
      CLKI        =>  clk_25mhz,
      CLKOP       =>  open,
      CLKOS       =>  open,
      CLKOS2      =>  clk_6M,
      CLKOS3      =>  open
  );

  wifi_rxd <= ftdi_txd;
  ftdi_rxd <= wifi_txd;

  S_enable <= not btn(1); -- BTN1 to hold OLED display

  G_oled: if C_oled generate
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
  end generate;
  
  -- APPLE ][ --

  --reset <= (not btn(0)) or power_on_reset;
  reset <= btn(2) or power_on_reset;

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
 
  COLOR_LINE_CONTROL <= COLOR_LINE and SW(3);  -- Color or B&W mode

  core : entity work.apple2
  port map
  (
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
    PDL_STROBE     => PDL_STROBE,
    GAMEPORT       => GAMEPORT,
    IO_SELECT      => IO_SELECT,
    DEVICE_SELECT  => DEVICE_SELECT,
    pcDebugOut     => cpu_pc,
    speaker        => speaker
    );
  audio_l(0) <= speaker;
  audio_l(3 downto 1) <= (others => '0');
  audio_r(0) <= speaker;
  audio_r(3 downto 1) <= (others => '0');
  audio_v(3 downto 0) <= (others => '0');

  clk_usb <= clk_6M; -- 48MHz full speed, 6MHz low speed

  G_us2: if C_joy_us2 generate
  usb_fpga_pu_dp <= '0';
  usb_fpga_pu_dn <= '0';
  us2_hid_host_inst: entity usbh_host_hid
  generic map
  (
    C_report_length_strict => '1',
    C_usb_speed => '0' -- '0':Low-speed '1':Full-speed
  )
  port map
  (
    clk => clk_usb, -- 6 MHz for low-speed USB1.0 device or 48 MHz for full-speed USB1.1 device
    bus_reset => '0',
    usb_dif => usb_fpga_dp,
    usb_dp  => usb_fpga_bd_dp,
    usb_dn  => usb_fpga_bd_dn,
    hid_report => S_hid_report,
    hid_valid => S_hid_report_valid
  );
  end generate;
  
  G_us3: if C_joy_us3 generate
  us3_fpga_pu_dp <= '0';
  us3_fpga_pu_dn <= '0';
  --us3_fpga_dp <= not us3_fpga_n_dp; -- pins up flat cable
  us3_fpga_dp <= 'Z'; -- pins down flat cable
  us3_hid_host_inst: entity usbh_host_hid
  generic map
  (
    C_report_length_strict => '1',
    C_usb_speed => '0' -- '0':Low-speed '1':Full-speed
  )
  port map
  (
    clk => clk_usb, -- 6 MHz for low-speed USB1.0 device or 48 MHz for full-speed USB1.1 device
    bus_reset => '0',
    usb_dif => us3_fpga_dp,
    usb_dp  => us3_fpga_bd_dp,
    usb_dn  => us3_fpga_bd_dn,
    hid_report => S_hid_report,
    hid_valid => S_hid_report_valid
  );
  end generate;

  G_us4: if C_joy_us4 generate
  us4_fpga_pu_dp <= '0';
  us4_fpga_pu_dn <= '0';
  --us4_fpga_dp <= not us4_fpga_n_dp; -- pins up flat cable
  us4_fpga_dp <= 'Z'; -- pins down flat cable
  us4_hid_host_inst: entity usbh_host_hid
  generic map
  (
    C_report_length_strict => '1',
    C_usb_speed => '0' -- '0':Low-speed '1':Full-speed
  )
  port map
  (
    clk => clk_usb, -- 6 MHz for low-speed USB1.0 device or 48 MHz for full-speed USB1.1 device
    bus_reset => '0',
    usb_dif => us4_fpga_dp,
    usb_dp  => us4_fpga_bd_dp,
    usb_dn  => us4_fpga_bd_dn,
    hid_report => S_hid_report,
    hid_valid => S_hid_report_valid
  );
  end generate;

  usbhid_report_decoder: entity usbhid_report_decoder
  port map
  (
    clk => clk_usb,
    hid_report => S_hid_report,
    hid_valid => S_hid_report_valid,
    decoded => S_hid_report_decoded
  );

  paddle : entity work.paddle
  port map(
    CLK_14M    => CLK_14M,
    PDL0       => S_hid_report_decoded.lstick_x,
    PDL1       => S_hid_report_decoded.lstick_y,
    PDL2       => S_hid_report_decoded.rstick_x,
    PDL3       => S_hid_report_decoded.rstick_y,
    PDL_STROBE => PDL_STROBE,
    GAMEPORT   => GAMEPORT(7 downto 4)
  );
  GAMEPORT(3 downto 0) <=
  (
    (S_hid_report_decoded.btn_rbumper  or S_hid_report_decoded.btn_start) &
    (S_hid_report_decoded.btn_ltrigger or S_hid_report_decoded.btn_b or S_hid_report_decoded.btn_x) &
    (S_hid_report_decoded.btn_rtrigger or S_hid_report_decoded.btn_a or S_hid_report_decoded.btn_y)
  ) & "0"; -- last 0 is for cassette (not used, we have floppy)

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

  G_kbd_us2: if C_kbd_us2 generate
  keyboard : entity work.keyboard port map (
    PS2_Clk  => usb_fpga_bd_dp,
    PS2_Data => usb_fpga_bd_dn,
    CLK_14M  => CLK_14M,
    reset    => reset,
    read     => read_key,
    K        => K
    );
  usb_fpga_pu_dp <= '1';
  usb_fpga_pu_dn <= '1';
  end generate;

  G_kbd_us3: if C_kbd_us3 generate
  keyboard : entity work.keyboard port map (
    PS2_Clk  => us3_fpga_bd_dp,
    PS2_Data => us3_fpga_bd_dn,
    CLK_14M  => CLK_14M,
    reset    => reset,
    read     => read_key,
    K        => K
    );
  us3_fpga_pu_dp <= '1';
  us3_fpga_pu_dn <= '1';
  end generate;

  G_kbd_us4: if C_kbd_us4 generate
  keyboard : entity work.keyboard port map (
    PS2_Clk  => us4_fpga_bd_dp,
    PS2_Data => us4_fpga_bd_dn,
    CLK_14M  => CLK_14M,
    reset    => reset,
    read     => read_key,
    K        => K
    );
  us4_fpga_pu_dp <= '1';
  us4_fpga_pu_dn <= '1';
  end generate;

  G_kbd_esp32: if C_kbd_esp32 generate
  keyboard : entity work.keyboard port map (
    PS2_Clk  => wifi_gpio22,
    PS2_Data => wifi_gpio21,
    CLK_14M  => CLK_14M,
    reset    => reset,
    read     => read_key,
    K        => K
    );
  end generate;

  G_yes_apple2_disk: if C_apple2_disk generate
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
  led(0) <= D1_ACTIVE;
  led(1) <= D2_ACTIVE;
  end generate; -- apple2_disk
  
  G_not_apple2_disk: if not C_apple2_disk generate
  TRACK(4 downto 0) <= btn(6 downto 2);
  end generate; -- not apple2_disk, tracks selected by BTNs

  G_apple2_sdcard: if C_sdcard generate
  sdcard_interface : entity work.spi_controller port map (
    CLK_14M        => CLK_14M,
    RESET          => RESET,

    CS_N           => sd_d(3),
    SCLK           => sd_clk,
    MOSI           => sd_cmd,
    MISO           => sd_d(0),

    SDHC           => led(2),

    track          => TRACK,
    image          => image,

    ram_write_addr => TRACK_RAM_ADDR,
    ram_di         => TRACK_RAM_DI,
    ram_we         => TRACK_RAM_WE
    );
  sd_d(1) <= 'Z';
  sd_d(2) <= 'Z';
  S_oled(45 downto 32) <= TRACK_RAM_ADDR;
  S_oled(55 downto 48) <= TRACK_RAM_DI;
  -- selects disk image
  --image <= "0000000" & SW(2 downto 0);
  image <= "000000" & x"0";
  end generate; -- apple2_sdcard

  G_disk2_spi_slave: if C_esp32 generate
  B_disk2_spi_slave: block
    signal spi_rd, spi_wr: std_logic;
    signal spi_addr: std_logic_vector(31 downto 0);
    signal spi_data_out, spi_data_in: std_logic_vector(7 downto 0);
    signal R_btn, R_btn_latch: std_logic_vector(btn'range);
    signal R_track : unsigned(track'range);
    signal R_irq: std_logic_vector(1 downto 0); -- interrupt request register
    signal R_track_irq, R_btn_irq: std_logic;
    signal R_spi_rd: std_logic;
    signal R_btn_debounce: unsigned(19 downto 0); -- 26Hz btn debounce
  begin
  E_disk2_spi_slave: entity work.spirw_slave
  generic map
  (
    c_addr_bits => 32
  )
  port map
  (
    clk                 => CLK_14M,

    csn                 => spi_csn,
    sclk                => wifi_gpio26, --              -- sd_clk,   -- wifi_gpio14
    mosi                => sd_d(1),     -- wifi_gpio4,  -- sd_cmd,   -- wifi_gpio15
    miso                => sd_d(2),     -- wifi_gpio12, -- sd_d(0),  -- wifi_gpio2

    rd                  => spi_rd,
    wr                  => spi_wr,
    addr                => spi_addr,
    data_in             => spi_data_in,
    data_out            => spi_data_out
  );
  TRACK_RAM_WE <= '1' when spi_wr = '1' and spi_addr(31 downto 30) = "00" else '0'; -- write disk track to 0x0000
  TRACK_RAM_ADDR <= unsigned(spi_addr(TRACK_RAM_ADDR'range));
  track_ram_di <= unsigned(spi_data_out);
  -- read: 0x0000 track and irq state, 0xFE00 btn state
  --spi_data_in <= std_logic_vector("00" & TRACK) when spi_addr(15 downto 14) = "00" else "0" & btn;
  --spi_data_in <= std_logic_vector(R_btn_irq & R_track_irq & TRACK) when spi_addr(15 downto 14) = "00" else "0" & btn;
  sd_d(3) <= 'Z'; -- CS_N
  sd_d(1) <= 'Z';
  sd_d(2) <= 'Z';
  S_oled(45 downto 32) <= TRACK_RAM_ADDR; -- disk writes to track buffer
  --S_oled(45 downto 32) <= TRACK_ADDR; -- CPU reads from track buffer
  S_oled(55 downto 48) <= TRACK_RAM_DI;
  P_read_irq_flags: process(CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      if spi_rd = '1' then
        case spi_addr(31 downto 30) is
          when "00" => -- reading track number resets IRQ state
            spi_data_in <= std_logic_vector(R_btn_irq & R_track_irq & TRACK);
          when others =>
            spi_data_in <= "0" & R_btn;
        end case;
      end if;
    end if;
  end process;
  -- generate track change request signal and track BTN state
  P_irq_controller: process(CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      R_spi_rd <= spi_rd;
      if spi_rd = '0' and R_spi_rd = '1' and spi_addr(31 downto 30) = "00" then -- reading track number resets IRQ state
        R_track_irq <= '0';
        R_btn_irq <= '0';
      else
        if R_track /= track then
          R_track_irq <= '1';
        end if;
        R_track <= track;
        R_btn_latch <= btn;
        if R_btn /= R_btn_latch and R_btn_debounce(R_btn_debounce'high) = '1' and R_btn_irq = '0' then
          R_btn_irq <= '1';
          R_btn_debounce <= (others => '0');
          R_btn <= R_btn_latch;
        else
          if R_btn_debounce(R_btn_debounce'high) = '0' then
            R_btn_debounce <= R_btn_debounce + 1;
          end if;
        end if;
      end if;
    end if;
  end process;
  wifi_gpio0 <= not (R_track_irq or R_btn_irq); -- interrupt line, active on falling edge
  end block;
  end generate; -- disk2_spi_slave

  ram_64K: entity work.bram_true2p_1clk
  generic map
  (
        dual_port => true,
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
        data_out_a => bram_dq,
        we_b => btn(3), -- press BTN3 to spoil CRC of warmstart reset vector
        addr_b => x"03F4", -- reset vector CRC address 0x03F4
        data_in_b => std_logic_vector(flash_clk(7 downto 0)) -- random
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
  vga_hsync <= not vga_hs;
  vga_vsync <= not vga_vs;
  spi_csn <= not wifi_gpio19;

  -- SPI OSD pipeline
  spi_osd_inst: entity work.spi_osd
  generic map
  (
    c_start_x => 26, c_start_y => 32, -- xy centered
    c_chars_x => 64, c_chars_y => 20, -- xy size, slightly less than full screen
    c_init_on => 0, -- 1:OSD initially shown without any SPI init
    c_char_file => "osd.mem", -- initial OSD content
    c_font_file => "font_bizcat8x16.mem"
  )
  port map
  (
    clk_pixel => clk_pixel, clk_pixel_ena => '1',
    i_r => std_logic_vector(vga_r(9 downto 2)),
    i_g => std_logic_vector(vga_g(9 downto 2)),
    i_b => std_logic_vector(vga_b(9 downto 2)),
    i_hsync => vga_hsync, i_vsync => vga_vsync, i_blank => vga_blank,
    i_csn => spi_csn, i_sclk => wifi_gpio26, i_mosi => sd_d(1), o_miso => open,
    o_r => osd_vga_r, o_g => osd_vga_g, o_b => osd_vga_b,
    o_hsync => osd_vga_hsync, o_vsync => osd_vga_vsync, o_blank => osd_vga_blank
  );

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

      in_red   => osd_vga_r,
      in_green => osd_vga_g,
      in_blue  => osd_vga_b,

      in_hsync => osd_vga_hsync,
      in_vsync => osd_vga_vsync,
      in_blank => osd_vga_blank,

      -- single-ended output ready for differential buffers
      out_red   => dvid_red,
      out_green => dvid_green,
      out_blue  => dvid_blue,
      out_clock => dvid_clock
  );

  -- vendor specific DDR modules
  -- convert SDR 2-bit input to DDR clocked 1-bit output (single-ended)
  ddr_clock: ODDRX1F port map (D0=>dvid_clock(0), D1=>dvid_clock(1), Q=>gpdi_dp(3), SCLK=>clk_pixel_shift, RST=>'0');
  ddr_red:   ODDRX1F port map (D0=>dvid_red(0),   D1=>dvid_red(1),   Q=>gpdi_dp(2), SCLK=>clk_pixel_shift, RST=>'0');
  ddr_green: ODDRX1F port map (D0=>dvid_green(0), D1=>dvid_green(1), Q=>gpdi_dp(1), SCLK=>clk_pixel_shift, RST=>'0');
  ddr_blue:  ODDRX1F port map (D0=>dvid_blue(0),  D1=>dvid_blue(1),  Q=>gpdi_dp(0), SCLK=>clk_pixel_shift, RST=>'0');

end Behavioral;
