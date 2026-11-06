using NUnit.Framework;
using Robust.Client.Graphics;
using Robust.Client.Graphics.Clyde;
using Robust.Client.ResourceManagement;
using Robust.Shared;
using Robust.Shared.Maths;
using Robust.UnitTesting;

namespace Robust.Client.IntegrationTests.ResourceManagement;

[TestFixture]
public sealed class HeadlessPreloadTest : RobustIntegrationTest
{
    [Test]
    public async Task PreloadRsiWithoutGraphicsContext()
    {
        using var client = StartClient(new ClientIntegrationOptions
        {
            Pool = false,
#if FULL_RELEASE
            // Full releases mount resources beside their executable. The test
            // executable instead uses the same engine resources as Debug tests.
            Options = new GameControllerOptions
            {
                ResourceMountDisabled = true,
                LoadContentResources = false,
                LoadConfigAndUserData = false,
                MountOptions = new MountOptions
                {
                    DirMounts = [Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "../../Resources"))]
                }
            }
#endif
        });
        await client.WaitIdleAsync();
        await client.WaitAssertion(() =>
        {
            Assert.That(client.Resolve<IClyde>(), Is.TypeOf<ClydeHeadless>());
            var cache = client.Resolve<IResourceCache>();
            var rsi = cache.GetAllResources<RSIResource>()
                .Single(pair => pair.Key.ToString() == "/Textures/debugRotation.rsi").Value.RSI;
            Assert.That(rsi.Size, Is.EqualTo(new Vector2i(32, 32)));
            Assert.That(rsi.TryGetState("direction4", out _), Is.True);
        });
    }
}
