import { TokenBridge } from "../src/core/tokenBridge";
import { Logger } from "../src/core/tokenBridge.utils";

async function demonstrateTokenPayments() {
  console.log('Starting SYNR TokenBridge Demonstration\n');

  const bridge = new TokenBridge({
    rpcUrl: "https://api.devnet.solana.com",
    simulate: true,
    networkName: "devnet",
    logger: new Logger(true)
  });

  try {
    const health = await bridge.healthCheck();
    console.log('Bridge Health:', health);

    const robotAddress = "robotWallet123456789";
    const balance = await bridge.checkBalance(robotAddress);
    console.log('Robot Balance:', balance);

    console.log('\nMaking payment for model sync...');
    const paymentResult = await bridge.payForSync({
      robotId: "humanoidBotA1",
      payer: robotAddress,
      amount: 25.5,
      memo: "AI model synchronization service"
    });

    console.log('Payment Result:', paymentResult);

    if (paymentResult.txHash) {
      console.log('\nVerifying transaction...');
      const verification = await bridge.verifyTransaction(paymentResult.txHash);
      console.log('Verification Result:', verification);
    }

    console.log('\nDemonstrating token transfer...');
    const transferResult = await bridge.transferToken({
      robotId: "armBot02",
      payer: robotAddress,
      receiver: "warehouseBotB7",
      amount: 10.2,
      memo: "Inter-robot service payment"
    });

    console.log('Transfer Result:', transferResult);

    const networkInfo = bridge.getNetworkInfo();
    console.log('\nNetwork Information:', networkInfo);
    console.log('SDK Version:', bridge.getVersion());

    bridge.on('paid', (data) => {
      console.log('\nEVENT: Payment completed:', data);
    });

    bridge.on('balance_checked', (data) => {
      console.log('EVENT: Balance checked:', data);
    });

    await bridge.payForSync({
      robotId: "droneBotC3", 
      payer: robotAddress,
      amount: 5.0,
      memo: "Navigation data sync"
    });

  } catch (error) {
    console.error('Demonstration failed:', error);
  } finally {
    await bridge.disconnect();
    console.log('\nDemonstration completed - Bridge disconnected');
  }
}

if (require.main === module) {
  demonstrateTokenPayments().catch(console.error);
}

export { demonstrateTokenPayments };