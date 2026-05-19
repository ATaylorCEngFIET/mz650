library ieee;
use ieee.std_logic_1164.all;
use ieee.fixed_pkg.all;
use std.env.all;

entity tb_dft8_spectrum is
end entity tb_dft8_spectrum;

architecture sim of tb_dft8_spectrum is
  constant WIDTH      : positive := 16;
  constant CLK_PERIOD : time := 10 ns;

  type sample_array_t is array (natural range <>) of integer;
  constant INPUT_SAMPLES      : sample_array_t(0 to 7) := (4, 3, 0, -3, -4, -3, 0, 3);
  constant MIN_DOMINANT_BIN_1 : integer := 12;
  constant MAX_OTHER_BINS     : integer := 1;

  signal clk          : std_logic := '0';
  signal rst          : std_logic := '1';
  signal sample_valid : std_logic := '0';
  signal sample_i     : sfixed(WIDTH - 1 downto 0) := (others => '0');
  signal output_valid : std_logic;
  signal bin_index_o  : ufixed(2 downto 0);
  signal magnitude_o  : ufixed(31 downto 0);
begin
  clk <= not clk after CLK_PERIOD / 2;

  dut : entity work.dft8_spectrum
    generic map (
      WIDTH => WIDTH
    )
    port map (
      clk          => clk,
      rst          => rst,
      sample_valid => sample_valid,
      sample_i     => sample_i,
      output_valid => output_valid,
      bin_index_o  => bin_index_o,
      magnitude_o  => magnitude_o
    );

  stimulus : process
    variable outputs_seen : integer := 0;
  begin
    wait for 30 ns;
    rst <= '0';

    for idx in INPUT_SAMPLES'range loop
      sample_i <= to_sfixed(real(INPUT_SAMPLES(idx)), sample_i'high, sample_i'low);
      sample_valid <= '1';
      wait until rising_edge(clk);
    end loop;
    sample_valid <= '0';

    while outputs_seen < 4 loop
      wait until rising_edge(clk);
      wait for 1 ns;
      if output_valid = '1' then
        assert to_integer(bin_index_o) = outputs_seen
          report "Unexpected DFT bin order. Expected " & integer'image(outputs_seen) &
                 ", got " & integer'image(to_integer(bin_index_o))
          severity failure;
        if outputs_seen = 1 then
          assert to_integer(magnitude_o) >= MIN_DOMINANT_BIN_1
            report "DFT dominant bin is too small. Got " & integer'image(to_integer(magnitude_o))
            severity failure;
        else
          assert to_integer(magnitude_o) <= MAX_OTHER_BINS
            report "Unexpected leakage magnitude for bin " & integer'image(outputs_seen) &
                   ". Got " & integer'image(to_integer(magnitude_o))
            severity failure;
        end if;
        outputs_seen := outputs_seen + 1;
      end if;
    end loop;

    report "tb_dft8_spectrum PASSED" severity note;
    stop;
    wait;
  end process;
end architecture sim;
